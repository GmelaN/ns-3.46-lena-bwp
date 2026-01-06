// Scenario: BWP switch based on DL queue length threshold at gNB.
// Uses BSR reports from BwpManagerGnb and forces BWP 0/1 via ForceAttempt.

#include "ns3/applications-module.h"
#include "ns3/bwp-manager-gnb.h"
#include "ns3/bwp-manager-ue.h"
#include "ns3/callback.h"
#include "ns3/cc-bwp-helper.h"
#include "ns3/command-line.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-epc-helper.h"
#include "ns3/nr-epc-tft.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"

#include <cmath>
#include <unordered_map>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BwpSwitchQueueThresholdExample");

static void ForceAttempt(Ptr<NrUeNetDevice> ueDev, Ptr<BwpManagerGnb> gnbMgr, uint8_t targetBwp);
static void ForceBothSides(Time delay, Ptr<NrUeNetDevice> ueDev, Ptr<BwpManagerGnb> gnbMgr, uint8_t targetBwp);

static void
ForceAttempt(Ptr<NrUeNetDevice> ueDev, Ptr<BwpManagerGnb> gnbMgr, uint8_t targetBwp)
{
    Ptr<NrUePhy> uePhy = ueDev->GetPhy(0);
    Ptr<BwpManagerUe> ueBwp = ueDev->GetBwpManager();
    uint16_t rnti = uePhy->GetRnti();
    if (rnti == 0)
    {
        Simulator::Schedule(MilliSeconds(1), &ForceAttempt, ueDev, gnbMgr, targetBwp);
        return;
    }
    ueBwp->ForceActiveBwp(targetBwp);
    gnbMgr->ForceUeBwp(rnti, targetBwp);
    NS_LOG_INFO("Force UE rnti=" << rnti << " to BWP " << +targetBwp << " at "
                                   << Simulator::Now().GetSeconds() << "s");
}

static void
ForceBothSides(Time delay, Ptr<NrUeNetDevice> ueDev, Ptr<BwpManagerGnb> gnbMgr, uint8_t targetBwp)
{
    NS_ABORT_MSG_IF(gnbMgr == nullptr, "BwpManagerGnb not found");
    Simulator::Schedule(delay, &ForceAttempt, ueDev, gnbMgr, targetBwp);
}

static void
ToggleTrafficPattern(Ptr<OnOffApplication> app,
                     bool burst,
                     Time period,
                     DataRate burstRate,
                     DataRate nonBurstRate)
{
    if (app == nullptr)
    {
        return;
    }

    DataRate nextRate = burst ? burstRate : nonBurstRate;
    app->SetAttribute("DataRate", DataRateValue(nextRate));
    NS_LOG_UNCOND("--------------------Traffic pattern -> " << (burst ? "burst" : "non-burst") << " at "
                                        << Simulator::Now().GetSeconds() << "s--------------------");

    Simulator::Schedule(period,
                        &ToggleTrafficPattern,
                        app,
                        !burst,
                        period,
                        burstRate,
                        nonBurstRate);
}

static double
GetP99DelayMs(const Histogram& hist, uint32_t samples)
{
    if (samples == 0)
    {
        return 0.0;
    }

    uint32_t nBins = hist.GetNBins();
    if (nBins == 0)
    {
        return 0.0;
    }

    uint64_t target = static_cast<uint64_t>(std::ceil(samples * 0.99));
    uint64_t cumulative = 0;
    for (uint32_t i = 0; i < nBins; ++i)
    {
        cumulative += hist.GetBinCount(i);
        if (cumulative >= target)
        {
            return hist.GetBinEnd(i) * 1000.0;
        }
    }

    return hist.GetBinEnd(nBins - 1) * 1000.0;
}

struct SwitchContext
{
    Ptr<NrUeNetDevice> ueDev;
    Ptr<BwpManagerGnb> gnbMgr;
    uint32_t threshold;
    std::unordered_map<uint16_t, uint8_t> lastTarget;
    std::unordered_map<uint16_t, Time> lastSwitch;
    std::vector<std::pair<double, uint8_t>> switchTimeline;
};

static SwitchContext* g_ctx = nullptr;

static void
DlBsrCallback(uint16_t rnti,
              uint8_t lcid,
              uint32_t txQueueSize,
              uint32_t retxQueueSize,
              uint16_t statusPduSize)
{
    (void)lcid;
    (void)retxQueueSize;
    (void)statusPduSize;

    if (g_ctx == nullptr || g_ctx->ueDev == nullptr || g_ctx->gnbMgr == nullptr)
    {
        return;
    }

    uint16_t ueRnti = g_ctx->ueDev->GetPhy(0)->GetRnti();
    if (ueRnti == 0 || ueRnti != rnti)
    {
        return;
    }

    uint32_t queueBytes = txQueueSize;

    uint8_t targetBwp = (queueBytes >= g_ctx->threshold) ? 1 : 0;
    // uint8_t targetBwp = 0;

    auto lastIt = g_ctx->lastTarget.find(rnti);
    if (lastIt != g_ctx->lastTarget.end() && lastIt->second == targetBwp)
    {
        // NS_LOG_UNCOND("CANNOT FIND TARGET FROM CONTEXT. RETURN.");
        return;
    }

    Time now = Simulator::Now();
    auto holdIt = g_ctx->lastSwitch.find(rnti);
    if (holdIt != g_ctx->lastSwitch.end() &&
        (now - holdIt->second) < MilliSeconds(100))
    {
        return;
    }

    g_ctx->lastTarget[rnti] = targetBwp;
    g_ctx->lastSwitch[rnti] = now;
    if (g_ctx->switchTimeline.empty() || g_ctx->switchTimeline.back().second != targetBwp)
    {
        g_ctx->switchTimeline.push_back({now.GetSeconds(), targetBwp});
    }
    NS_LOG_UNCOND("DL queue bytes=" << queueBytes << " (threshold=" << g_ctx->threshold
                                    << ") -> target BWP " << +targetBwp << " at "
                                    << Simulator::Now().GetSeconds() << "s");

    ForceAttempt(g_ctx->ueDev, g_ctx->gnbMgr, targetBwp);
}

int
main(int argc, char* argv[])
{
    uint32_t queueThreshold = 3000; // bytes
    double simTime = 5.0;
    uint32_t patternPeriodMs = 500;
    double switchDelay = 20.0;
    std::string burstRate = "100Mbps";
    std::string nonBurstRate = "0.1Mbps";

    CommandLine cmd;
    cmd.AddValue("queueThreshold",
                 "DL queue size threshold in bytes for switching to BWP1",
                 queueThreshold);
    cmd.AddValue("patternPeriodMs", "Burst/non-burst toggle period in ms", patternPeriodMs);
    cmd.AddValue("burstRate", "Burst traffic rate", burstRate);
    cmd.AddValue("nonBurstRate", "Non-burst traffic rate", nonBurstRate);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.Parse(argc, argv);

    LogComponentEnableAll(LOG_PREFIX_TIME);
    // LogComponentEnable("BwpSwitchQueueThresholdExample", LOG_LEVEL_INFO);

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetSchedulerTypeId(NrMacSchedulerTdmaRR::GetTypeId());

    double bwp0Bandwidth = 5e6;
    double bwp1Bandwidth = 20e6;
    double totalBandwidth = bwp0Bandwidth + bwp1Bandwidth;

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf;
    bandConf.m_centralFrequency = 3.5e9;
    bandConf.m_channelBandwidth = totalBandwidth;
    bandConf.m_numCc = 1;
    bandConf.m_numBwp = 2;
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    auto& bwp0 = band.GetBwpAt(0, 0);
    auto& bwp1 = band.GetBwpAt(0, 1);
    double bandLower = band.m_lowerFrequency;
    double bandHigher = band.m_higherFrequency;

    bwp0->m_lowerFrequency = bandLower;
    bwp0->m_higherFrequency = bandLower + bwp0Bandwidth - 1;
    bwp0->m_channelBandwidth = bwp0Bandwidth;
    bwp0->m_centralFrequency = bandLower + (bwp0Bandwidth / 2.0);

    bwp1->m_lowerFrequency = bwp0->m_higherFrequency + 1;
    bwp1->m_higherFrequency = bwp1->m_lowerFrequency + bwp1Bandwidth - 1;
    bwp1->m_channelBandwidth = bwp1Bandwidth;
    bwp1->m_centralFrequency = (bwp1->m_lowerFrequency + bwp1->m_higherFrequency) / 2.0;

    NS_ABORT_MSG_IF(bwp1->m_higherFrequency > bandHigher,
                    "BWP1 exceeds operation band upper frequency");

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->AssignChannelsToBands({band});
    auto allBwps = CcBwpCreator::GetAllBwps({band});

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHost;
    remoteHost.Create(1);
    InternetStackHelper internet;
    internet.Install(remoteHost);
    internet.Install(ueNodes);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2ph.SetChannelAttribute("Delay", StringValue("1ms"));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost.Get(0));
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    (void)internetIpIfaces;

    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(ueDevs);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost.Get(0)->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                               Ipv4Mask("255.0.0.0"),
                                               1);

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(i)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    uint16_t dlPort = 1234;
    PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    ApplicationContainer serverApps = dlPacketSinkHelper.Install(ueNodes.Get(0));

    OnOffHelper dlClient("ns3::UdpSocketFactory",
                         InetSocketAddress(ueIpIface.GetAddress(0), dlPort));
    dlClient.SetAttribute("DataRate", DataRateValue(DataRate(nonBurstRate)));
    dlClient.SetAttribute("PacketSize", UintegerValue(1400));
    dlClient.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    dlClient.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer clientApps = dlClient.Install(remoteHost.Get(0));
    Ptr<OnOffApplication> dlOnOff = clientApps.Get(0)->GetObject<OnOffApplication>();

    Ptr<NrEpcTft> tft = Create<NrEpcTft>();
    NrEpcTft::PacketFilter dlpf;
    dlpf.localPortStart = dlPort;
    dlpf.localPortEnd = dlPort;
    tft->Add(dlpf);
    NrEpsBearer bearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
    nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(0), bearer, tft);

    Ptr<NrUeNetDevice> ueDev = ueDevs.Get(0)->GetObject<NrUeNetDevice>();
    Ptr<BwpManagerUe> ueBwpMgr = ueDev->GetBwpManager();

    Ptr<NrGnbNetDevice> gnbDev = gnbDevs.Get(0)->GetObject<NrGnbNetDevice>();
    Ptr<BwpManagerGnb> gnbBwpMgr =
        DynamicCast<BwpManagerGnb>(gnbDev->GetComponentCarrierManager());

    ueBwpMgr->SetAttributeSwitchingDelay(MilliSeconds(switchDelay));
    gnbBwpMgr->SetAttributeSwitchingDelay(MilliSeconds(switchDelay));
    NS_ABORT_MSG_IF(gnbBwpMgr == nullptr, "BwpManagerGnb not found");

    ueBwpMgr->ForceActiveBwp(0);
    ForceBothSides(Seconds(0.0), ueDev, gnbBwpMgr, 0);

    SwitchContext ctx;
    ctx.ueDev = ueDev;
    ctx.gnbMgr = gnbBwpMgr;
    ctx.threshold = queueThreshold;
    ctx.switchTimeline.push_back({0.0, 0});

    g_ctx = &ctx;
    gnbBwpMgr->TraceConnectWithoutContext("DlBsrReport", MakeCallback(&DlBsrCallback));

    NS_LOG_UNCOND("Traffic pattern -> non-burst at " << Simulator::Now().GetSeconds() << "s");
    Simulator::Schedule(MilliSeconds(patternPeriodMs),
                        &ToggleTrafficPattern,
                        dlOnOff,
                        true,
                        MilliSeconds(patternPeriodMs),
                        DataRate(burstRate),
                        DataRate(nonBurstRate));

    serverApps.Start(MilliSeconds(100));
    clientApps.Start(MilliSeconds(100));
    serverApps.Stop(Seconds(simTime));
    clientApps.Stop(Seconds(simTime));

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    auto stats = monitor->GetFlowStats();
    for (const auto& kv : stats)
    {
        auto flowId = kv.first;
        auto st = kv.second;
        auto t = classifier->FindFlow(flowId);

        if (t.destinationPort != dlPort && t.sourcePort != dlPort)
        {
            continue;
        }
        if (st.rxPackets == 0)
        {
            continue;
        }
        double duration = st.timeLastRxPacket.GetSeconds() - st.timeFirstRxPacket.GetSeconds();
        if (duration <= 0)
        {
            continue;
        }

        double throughputMbps = (st.rxBytes * 8.0) / duration / 1e6;
        double meanDelayMs =
            (st.rxPackets > 0) ? (st.delaySum.GetSeconds() / st.rxPackets * 1000.0) : 0.0;
        double p99DelayMs = GetP99DelayMs(st.delayHistogram, st.rxPackets);

        NS_LOG_UNCOND("Flow " << flowId << " port=" << t.destinationPort
                              << " rxPkts=" << st.rxPackets << " throughput=" << throughputMbps
                              << " Mbps meanDelay=" << meanDelayMs
                              << " ms p99Delay=" << p99DelayMs << " ms");
    }

    double powerPerMHzMw = 20.0;
    std::vector<double> bwpPowerMw = {bwp0Bandwidth / 1e6 * powerPerMHzMw,
                                      bwp1Bandwidth / 1e6 * powerPerMHzMw};
    std::vector<double> timePerBwp(2, 0.0);
    double energyJ = 0.0;

    auto timeline = ctx.switchTimeline;
    if (timeline.empty())
    {
        timeline.push_back({0.0, 0});
    }
    timeline.push_back({simTime, timeline.back().second});
    for (size_t k = 1; k < timeline.size(); ++k)
    {
        double dt = timeline[k].first - timeline[k - 1].first;
        uint8_t bwp = timeline[k - 1].second;
        if (bwp < bwpPowerMw.size())
        {
            energyJ += dt * bwpPowerMw[bwp] * 1e-3;
            timePerBwp[bwp] += dt;
        }
    }

    size_t switchCount = (timeline.size() > 1) ? (timeline.size() - 2) : 0;
    NS_LOG_UNCOND("Energy estimate=" << energyJ << " J"
                                     << " timeBwp0=" << timePerBwp[0] << "s"
                                     << " timeBwp1=" << timePerBwp[1] << "s"
                                     << " switches=" << switchCount);

    Simulator::Destroy();

    return 0;
}
