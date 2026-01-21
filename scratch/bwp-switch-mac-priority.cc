// Scenario: 1 gNB with two BWPs (narrow/wide), four UEs forced to switch BWPs at different times.
// - UE0: stays on narrow for whole run
// - UE1: narrow until 0.3s, then wide
// - UE2: narrow until 0.7s, then wide
// - UE3: stays on wide for whole run
// Collects per-flow throughput and mean delay using FlowMonitor.

#include "ns3/applications-module.h"
#include "ns3/bwp-manager-gnb.h"
#include "ns3/bwp-manager-ue.h"
#include "ns3/cc-bwp-helper.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/config.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-epc-helper.h"
#include "ns3/nr-epc-tft.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-scheduler-ofdma-rr.h"
#include "ns3/nr-mac-sap.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-bwp-switch-controller.h"
#include "ns3/nr-bwp-switch-trigger-helper.h"
#include "ns3/nr-mac-scheduler-ns3.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/random-variable-stream.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/traffic-generator-ngmn-ftp-multi.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"

#include "ns3/command-line.h"

#include <unordered_map>
#include <unordered_set>
#include <sstream>
#include <cmath>
#include <vector>
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BwpSwitchMultiExample");

static uint32_t simTime = 10;
static uint8_t numUes = 30;
static bool enableInternalPolicy = true; // set true to use helper's built-in policy

// Timeline-based policy state shared with controller
static Ptr<NrBwpSwitchTriggerHelper> g_triggerHelper;
static Ptr<NrBwpSwitchController> g_bwpController;
static std::vector<double> g_bwpPowerMw = {50.0, 400.0}; // example mW for narrow/wide 5 / 40 MHz
static std::vector<double> g_bwpBandwidthHz = {5e6, 40e6};
static double g_spectralEfficiency = 1.0;
static uint8_t g_initialBwpId = 0; // default start on narrow BWP

static void
DlQueueTrace(uint16_t rnti, uint8_t lcid, uint32_t queueBytes, uint16_t bwpId)
{
    if (!g_triggerHelper)
    {
        return;
    }
    g_triggerHelper->NotifyDlQueue(rnti, lcid, static_cast<uint8_t>(bwpId), 0, queueBytes);
}


// Simple probe policy for the trigger helper that logs aggregated AoI/queue bins.
// static NrBwpSwitchDecision
// ProbeTriggerPolicy(const NrBwpSwitchState& state)
// {
//     NS_LOG_UNCOND("TriggerHelper probe rnti=" << state.bsr.rnti << " avgAoI=" << state.avgAoIMs
//                                               << "s bin=" << +state.aoiBin
//                                               << " avgQueue=" << state.avgQueueSizeBytes
//                                               << "B bin=" << +state.queueBin
//                                               << " currentBwp=" << +state.currentBwpId
//                                               << " priority=" << +state.priority);
//     return {};
// }


static std::unordered_set<uint16_t> g_pdcpUeConnected;
static std::unordered_set<uint16_t> g_pdcpGnbConnected;
struct EnergyState
{
    bool initialized{false};
    double lastTime{0.0};
    uint8_t currentBwp{0};
    double energyJ{0.0};
};
static std::unordered_map<uint16_t, EnergyState> g_energyByRnti;
static std::unordered_map<uint16_t, uint32_t> g_ueIndexByRnti;

static void
PdcpTxTraceDl(uint16_t rnti, uint8_t lcid, uint32_t size)
{
    auto it = g_energyByRnti.find(rnti);
    if (it == g_energyByRnti.end())
    {
        return;
    }
    auto& st = it->second;
    if (!st.initialized)
    {
        st.initialized = true;
        st.currentBwp = g_initialBwpId;
    }
    uint8_t bwp = st.currentBwp;
    if (bwp < g_bwpPowerMw.size() && bwp < g_bwpBandwidthHz.size() &&
        g_bwpBandwidthHz[bwp] > 0.0 && g_spectralEfficiency > 0.0)
    {
        double airTime = (8.0 * size) / (g_bwpBandwidthHz[bwp] * g_spectralEfficiency);
        double energyJ = g_bwpPowerMw[bwp] * 1e-3 * airTime;
        st.energyJ += energyJ;
    }
}

static void
PdcpRxTraceDl(uint16_t rnti, uint8_t lcid, uint32_t /*size*/, uint64_t delayNs)
{
    Time delay = NanoSeconds(delayNs);
    if (g_bwpController)
    {
        g_bwpController->RecordAoiSample(delay);
    }
    if (g_triggerHelper)
    {
        g_triggerHelper->RecordAoiSample(rnti, lcid, delay);
    }
}

static void
OnSwitchEnergy(uint16_t rnti, uint8_t fromBwp, uint8_t toBwp, double switchEnergyJ)
{
    auto& st = g_energyByRnti[rnti];
    st.initialized = true;
    st.lastTime = Simulator::Now().GetSeconds();
    st.currentBwp = toBwp;
    st.energyJ += switchEnergyJ;

    NS_LOG_INFO("SwitchEnergyTrace rnti=" << rnti << " from=" << +fromBwp << " to=" << +toBwp
                                          << " switchEnergyJ=" << switchEnergyJ);
}

static void
ConnectPdcpTracesUe(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    (void)imsi;
    (void)cellId;
    // Avoid duplicate connections per RNTI.
    if (!g_pdcpUeConnected.insert(rnti).second)
    {
        return;
    }
    std::string base = context.substr(0, context.rfind('/')); // strip "/RandomAccessSuccessful"
    Config::ConnectWithoutContextFailSafe(base + "/DataRadioBearerMap/*/NrPdcp/RxPDU",
                                          MakeCallback(&PdcpRxTraceDl));
}

static void
ConnectPdcpTracesGnb(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    (void)imsi;
    (void)cellId;
    if (!g_pdcpGnbConnected.insert(rnti).second)
    {
        return;
    }
    std::ostringstream base;
    base << context.substr(0, context.rfind('/')) << "/UeMap/" << static_cast<uint32_t>(rnti);
    Config::ConnectWithoutContextFailSafe(base.str() + "/DataRadioBearerMap/*/NrPdcp/TxPDU",
                                          MakeCallback(&PdcpTxTraceDl));
}

static void
RegisterUeWithController(const Ptr<NrBwpSwitchController>& ctrl,
                         Ptr<NrUeNetDevice> ueDev,
                         Ptr<BwpManagerGnb> gnbMgr,
                         uint32_t ueIndex)
{
    Ptr<NrUePhy> uePhy = ueDev->GetPhy(0);
    uint16_t rnti = uePhy->GetRnti();
    if (rnti == 0)
    {
        Simulator::Schedule(MilliSeconds(1),
                            &RegisterUeWithController,
                            ctrl,
                            ueDev,
                            gnbMgr,
                            ueIndex);
        return;
    }

    // Seed energy accounting with the initial BWP.
    g_energyByRnti[rnti] = EnergyState{true, Simulator::Now().GetSeconds(), g_initialBwpId, 0.0};
    g_ueIndexByRnti[rnti] = ueIndex;

    ctrl->AddUeManager(rnti, ueDev->GetBwpManager());
    ueDev->GetBwpManager()->ForceActiveBwp(g_initialBwpId);
    ueDev->GetBwpManager()->SetAttributeSwitchingDelay(MilliSeconds(10));
    gnbMgr->ForceUeBwp(rnti, g_initialBwpId);

    NS_LOG_INFO("Registered UE" << ueIndex << " rnti=" << rnti);
}

int
main(int argc, char* argv[])
{
    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("numUes", "Number of UEs", numUes);
    cmd.AddValue("initialBwp", "Initial BWP for all UEs (0=narrow, 1=wide)", g_initialBwpId);
    cmd.AddValue("enableQlearning", "Enable trigger helper built-in Q-learning policy", enableInternalPolicy);
    cmd.Parse(argc, argv);

    LogComponentEnableAll(LogLevel(LOG_PREFIX_TIME));
    LogComponentEnableAll(LogLevel(LOG_PREFIX_FUNC));
    // LogComponentEnable("BwpSwitchMultiExample", LOG_LEVEL_INFO);
    // LogComponentEnable("NrBwpSwitchTriggerHelper", LOG_LEVEL_INFO);
    // LogComponentEnable("BwpManagerGnb", LOG_LEVEL_INFO);

    // Clamp initial BWP to available BWPs (0..1).
    g_initialBwpId = std::min<uint8_t>(g_initialBwpId, 1);

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaRR::GetTypeId());
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(false));

    // Two CCs (treated as two BWPs for forcing): narrow 5 MHz, wide 40 MHz.
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf narrowConf(3.40e9, 5e6, 1);
    CcBwpCreator::SimpleOperationBandConf wideConf(3.50e9, 40e6, 1);
    OperationBandInfo bandNarrow = ccBwpCreator.CreateOperationBandContiguousCc(narrowConf);
    OperationBandInfo bandWide = ccBwpCreator.CreateOperationBandContiguousCc(wideConf);
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->AssignChannelsToBands({bandNarrow, bandWide});
    auto allBwps = CcBwpCreator::GetAllBwps({bandNarrow, bandWide});

    // Internet/EPC stack must be installed before attaching (attach activates EPS bearer).
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHost;
    remoteHost.Create(1);
    InternetStackHelper internet;
    internet.Install(remoteHost);
    internet.Install(ueNodes);

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    // Numerology per BWP: 0 for narrow (BWP0), 2 for wide (BWP1).
    // Ptr<NrGnbNetDevice> gnbDevCast = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(0));
    // NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->SetAttribute("Numerology", UintegerValue(0));
    // NrHelper::GetGnbPhy(gnbDevs.Get(0), 1)->SetAttribute("Numerology", UintegerValue(2));


    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2ph.SetChannelAttribute("Delay", StringValue("1ms"));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost.Get(0));
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);

    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(ueDevs);
    std::map<Ipv4Address, uint32_t> ueIpToIndex;
    for (uint32_t i = 0; i < ueIpIfaces.GetN(); ++i)
    {
        ueIpToIndex.emplace(ueIpIfaces.GetAddress(i), i);
    }
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

    Ptr<BwpManagerGnb> gnbBwpMgr =
        DynamicCast<BwpManagerGnb>(gnbDevs.Get(0)->GetObject<NrGnbNetDevice>()->GetComponentCarrierManager());
    // Apply a switching delay (e.g., 2 ms) to mimic guard time for both gNB/UE managers.
    gnbBwpMgr->SetAttributeSwitchingDelay(MilliSeconds(20));
    gnbBwpMgr->SetAttribute("TxPowerBwp0Mw", DoubleValue(g_bwpPowerMw.at(0)));
    gnbBwpMgr->SetAttribute("TxPowerBwp1Mw", DoubleValue(g_bwpPowerMw.at(1)));
    gnbBwpMgr->SetAttribute("TxBandwidthBwp0Hz", DoubleValue(g_bwpBandwidthHz.at(0)));
    gnbBwpMgr->SetAttribute("TxBandwidthBwp1Hz", DoubleValue(g_bwpBandwidthHz.at(1)));
    gnbBwpMgr->SetAttribute("TxSpectralEfficiency", DoubleValue(g_spectralEfficiency));


    g_triggerHelper = CreateObject<NrBwpSwitchTriggerHelper>();
    g_triggerHelper->SetAttribute("EnableInternalPolicy", BooleanValue(enableInternalPolicy));
    if(enableInternalPolicy)
    {
        NS_LOG_UNCOND("Q-LEARNING");
        g_triggerHelper->SetAttribute("InternalPolicyMode", EnumValue(NrBwpSwitchTriggerHelper::InternalPolicyMode::POLICY_Q_LEARNING));
    }
    else
    {
        NS_LOG_UNCOND("QUEUE_WINDOW_DETECTION");
        g_triggerHelper->SetAttribute("InternalPolicyMode", EnumValue(NrBwpSwitchTriggerHelper::InternalPolicyMode::POLICY_WINDOW_DETECT));
    }
    
    // Give enough time for RA/RRC to complete so enqueue/ACK/BSR land before first evaluation.
    g_triggerHelper->SetAttribute("StartDelay", TimeValue(MilliSeconds(60)));
    g_triggerHelper->SetAttribute("EvaluationPeriod", TimeValue(MilliSeconds(50)));
    // Push bins to react to even lighter load/latency so BWP1 becomes eligible sooner.
    g_triggerHelper->SetAttribute("QueueThreshold1", UintegerValue(1000));
    g_triggerHelper->SetAttribute("QueueThreshold2", UintegerValue(3000));
    g_triggerHelper->SetAttribute("AoIThreshold1", TimeValue(MilliSeconds(5)));
    g_triggerHelper->SetAttribute("AoIThreshold2", TimeValue(MilliSeconds(10)));
    // Bias learning toward AoI by reducing energy weight/scale and updating all UEs every round.
    g_triggerHelper->SetAttribute("PreferenceEnergy", DoubleValue(0.5));
    g_triggerHelper->SetAttribute("PreferenceAoI", DoubleValue(0.5));
    g_triggerHelper->SetAttribute("RewardEnergyNorm", DoubleValue(5.0)); // J scale
    g_triggerHelper->SetAttribute("RewardAoiNorm", DoubleValue(10.0));  // ms scale (~5 ms)
    g_triggerHelper->SetAttribute("Epsilon", DoubleValue(0.2));
    g_triggerHelper->SetAttribute("EpsilonMin", DoubleValue(0.02));
    g_triggerHelper->SetAttribute("EpsilonDecay", DoubleValue(5e-3));
    g_triggerHelper->SetAttribute("LearningRate", DoubleValue(0.05));
    g_triggerHelper->SetAttribute("EvalGroupModulo", UintegerValue(1));
    g_triggerHelper->SetAttribute("StaticPowerBwp0Mw", DoubleValue(g_bwpPowerMw.at(0)));
    g_triggerHelper->SetAttribute("StaticPowerBwp1Mw", DoubleValue(g_bwpPowerMw.at(1)));
    g_triggerHelper->SetGnbManager(gnbBwpMgr);
    // g_triggerHelper->SetPolicy(MakeCallback(&ProbeTriggerPolicy));
    for (uint32_t bwpIdx = 0; bwpIdx < allBwps.size(); ++bwpIdx)
    {
        Ptr<NrMacSchedulerNs3> sched =
            DynamicCast<NrMacSchedulerNs3>(NrHelper::GetScheduler(gnbDevs.Get(0), bwpIdx));
        if (sched)
        {
            sched->TraceConnectWithoutContext("DlBufferReport", MakeCallback(&DlQueueTrace));
        }
    }

    Ptr<NrBwpSwitchController> bwpController = CreateObject<NrBwpSwitchController>();
    g_bwpController = bwpController;

    g_triggerHelper->SetDecisionCallback(MakeCallback(&NrBwpSwitchController::OnHelperDecision, bwpController));
    bwpController->SetGnbManager(gnbBwpMgr);
    // bwpController->SetPolicy(MakeCallback(&TimelinePolicy));
    gnbBwpMgr->TraceConnectWithoutContext("BsrReport",
                                          MakeCallback(&NrBwpSwitchController::HandleBsr, bwpController));
    gnbBwpMgr->TraceConnectWithoutContext("SwitchEnergy", MakeCallback(&OnSwitchEnergy));

    // Applications: DL sinks per UE; traffic generated from remote host using NGMN FTP (ON/OFF).
    ApplicationContainer serverApps;
    Ptr<UniformRandomVariable> readingTimeRng = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> fileSizeMuRng = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> fileSizeSigmaRng = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> packetSizeRng = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> startJitterRng = CreateObject<UniformRandomVariable>();
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        uint16_t port = 5000 + i;
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        serverApps.Add(sinkHelper.Install(ueNodes.Get(i)));

        Ptr<NrEpcTft> tft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter pf;
        pf.localPortStart = port;
        pf.localPortEnd = port;
        tft->Add(pf);
        NrEpsBearer bearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
        nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(i), bearer, tft);

        Ptr<TrafficGeneratorNgmnFtpMulti> ftp = CreateObject<TrafficGeneratorNgmnFtpMulti>();
        ftp->SetRemote(InetSocketAddress(ueIpIfaces.GetAddress(i), port));
        ftp->SetProtocol(UdpSocketFactory::GetTypeId());

        // Less frequent, larger file transfers (~5 MB) to create bursty load.
        double readingMean = readingTimeRng->GetValue(0.1, 1.0); // seconds (0.1s to 1s)
        double baseFileSizeMu = std::log(50 * 1024 * 1024); // ln(bytes) for ~50 MB median
        double fileSizeMu = fileSizeMuRng->GetValue(baseFileSizeMu - 0.05, baseFileSizeMu + 0.05);
        double fileSizeSigma = fileSizeSigmaRng->GetValue(0.08, 0.12); // lognormal sigma
        ftp->SetAttribute("MaxFileSize", UintegerValue(50 * 1024 * 1024)); // 50 MB

        uint32_t packetSize = static_cast<uint32_t>(packetSizeRng->GetValue(900, 1100));
        ftp->SetAttribute("ReadingTimeMean", DoubleValue(readingMean));
        ftp->SetAttribute("FileSizeMu", DoubleValue(fileSizeMu));
        ftp->SetAttribute("FileSizeSigma", DoubleValue(fileSizeSigma));
        ftp->SetAttribute("PacketSize", UintegerValue(packetSize));

        // Stagger start slightly so bursts are de-synchronized across UEs.
        uint32_t startJitterMs = startJitterRng->GetInteger(0, 20);
        Time startTime = MilliSeconds(60 + startJitterMs);
        ftp->SetStartTime(startTime);
        ftp->SetStopTime(Seconds(simTime));
        remoteHost.Get(0)->AddApplication(ftp);
    }

    serverApps.Start(MilliSeconds(50));
    serverApps.Stop(Seconds(simTime));

    // Connect PDCP traces after RRC is ready (DL only: gNB Tx -> UE Rx) to feed AoI.
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/ConnectionReconfiguration",
                    MakeCallback(&ConnectPdcpTracesUe));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/ConnectionReconfiguration",
                    MakeCallback(&ConnectPdcpTracesGnb));

    for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ueDev = ueDevs.Get(i)->GetObject<NrUeNetDevice>();
        RegisterUeWithController(bwpController, ueDev, gnbBwpMgr, i);
    }

    // FlowMonitor for throughput/delay
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    struct PerUeAgg
    {
        double throughputMbpsSum{0.0};
        double meanDelayMsSum{0.0};
        uint32_t flows{0};
    };
    std::vector<PerUeAgg> ueAgg(numUes);

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    auto stats = monitor->GetFlowStats();
    Ipv4Address ueNetwork("7.0.0.0");
    Ipv4Mask ueMask("255.0.0.0");
    for (const auto& kv : stats)
    {
        auto flowId = kv.first;
        auto st = kv.second;
        auto t = classifier->FindFlow(flowId);

        // bool srcIsUe = (t.sourceAddress.CombineMask(ueMask) == ueNetwork);
        bool dstIsUe = (t.destinationAddress.CombineMask(ueMask) == ueNetwork);
        bool srcIsRemote = (t.sourceAddress == remoteHostAddr);
        bool dstIsRemote = (t.destinationAddress == remoteHostAddr);
        bool isDownlink = srcIsRemote && dstIsUe; // keep DL payload, drop UL ACKs

        // Keep only DL flows from remote host into UE subnet.
        if (!isDownlink)
        {
            continue;
        }

        double duration = st.timeLastRxPacket.GetSeconds() - st.timeFirstTxPacket.GetSeconds();
        if (duration <= 0)
        {
            continue;
        }
        double throughputMbps = (st.rxBytes * 8.0) / duration / 1e6;
        double meanDelayMs =
            (st.rxPackets > 0) ? (st.delaySum.GetSeconds() / st.rxPackets * 1000.0) : 0.0;

        uint32_t ueIdx = 0;
        auto it = ueIpToIndex.find(t.destinationAddress);
        if (it != ueIpToIndex.end())
        {
            ueIdx = it->second;
        }

        ueAgg.at(ueIdx).throughputMbpsSum += throughputMbps;
        ueAgg.at(ueIdx).meanDelayMsSum += meanDelayMs;
        ueAgg.at(ueIdx).flows++;

        NS_LOG_UNCOND("Flow " << flowId << " UE" << ueIdx
                              << " proto=" << static_cast<uint16_t>(t.protocol)
                              << " src=" << t.sourceAddress << ":" << t.sourcePort << " -> "
                              << t.destinationAddress << ":" << t.destinationPort
                              << " rxPkts=" << st.rxPackets << " throughput=" << throughputMbps
                              << " Mbps meanDelay=" << meanDelayMs << " ms");
    }

    double totalAvgThr = 0.0;
    double totalAvgDelay = 0.0;
    uint32_t ueCountWithFlow = 0;
    for (uint32_t i = 0; i < ueAgg.size(); ++i)
    {
        if (ueAgg[i].flows == 0)
        {
            NS_LOG_UNCOND("UE" << i << " avgThroughput=0 Mbps avgDelay=0 ms (no DL flow)");
            continue;
        }
        double avgThr = ueAgg[i].throughputMbpsSum / ueAgg[i].flows;
        double avgDelay = ueAgg[i].meanDelayMsSum / ueAgg[i].flows;
        totalAvgThr += avgThr;
        totalAvgDelay += avgDelay;
        ueCountWithFlow++;
        NS_LOG_UNCOND("UE" << i << " avgThroughput=" << avgThr << " Mbps avgDelay=" << avgDelay
                           << " ms");
    }


    // Energy estimate from PDCP TX bytes using fixed spectral efficiency.
    double energySum = 0.0;
    uint32_t energyCount = 0;
    for (auto& kv : g_energyByRnti)
    {
        uint16_t rnti = kv.first;
        auto& st = kv.second;
        uint32_t ueIdx = 0;
        auto it = g_ueIndexByRnti.find(rnti);
        if (it != g_ueIndexByRnti.end())
        {
            ueIdx = it->second;
        }
        energySum += st.energyJ;
        energyCount++;
        NS_LOG_UNCOND("UE" << ueIdx << " rnti=" << rnti << " energy_est=" << st.energyJ
                           << " J");
    }

    if (ueCountWithFlow > 0)
    {
        NS_LOG_UNCOND("\n\nAverage Throughput: " << (totalAvgThr / ueCountWithFlow)
                                                << " Mbps\tDelay: " << (totalAvgDelay / ueCountWithFlow)
                                                << " ms");
    }
    if (energyCount > 0)
    {
        NS_LOG_UNCOND("Average Energy: " << (energySum / energyCount) << " J");
    }

    // Aggregate AoI observed via PDCP enqueue/ack tracing.
    double avgAoiMs = bwpController->GetAverageAoISeconds() * 1000.0;
    NS_LOG_UNCOND("Average AoI: " << avgAoiMs << " ms");

    Simulator::Destroy();
    return 0;
}
