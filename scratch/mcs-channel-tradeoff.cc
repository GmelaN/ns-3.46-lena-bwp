// Scenario: Channel alternates between good/bad states while comparing
// (A) 3.5GHz, 100MHz, low MCS vs (B) 700MHz, 20MHz, mid MCS.
// Outputs periodic throughput, BLER, and retransmission rate.

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-module.h"
#include "ns3/nr-mac-scheduler-ofdma-rr.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/cc-bwp-helper.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/bwp-manager-gnb.h"
#include "ns3/bwp-manager-ue.h"
#include "ns3/point-to-point-helper.h"

#include <iomanip>
#include <algorithm>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("McsChannelTradeoff");

struct Counters
{
    uint64_t tbTotal{0};
    uint64_t tbError{0};
    uint64_t harqTotal{0};
    uint64_t harqNack{0};
    uint64_t harqRetx{0};
    uint64_t prbTotal{0};
};

static uint32_t g_numUes = 2;
static double g_simTime = 10.0;
static double g_appStart = 0.5;
static double g_statsPeriod = 0.1; // seconds
static double g_channelPeriod = 0.5; // seconds
static bool g_startGood = true;
static double g_goodDistance = 30.0;
static double g_badDistance = 1000.0;
static double g_ueHeight = 1.5;
static double g_gnbHeight = 10.0;
static double g_gnbTxPowerDbm = 5.0;

static double g_highFreqHz = 3.5e9;
static double g_highBwHz = 100e6;
static double g_lowFreqHz = 0.7e9;
static double g_lowBwHz = 20e6;
static uint8_t g_lowMcs = 4;
static uint8_t g_midMcs = 12;

static std::string g_errorModel = "ns3::NrEesmCcT1";

// Traffic
static double g_burstRateMbps = 400.0;
static uint32_t g_burstPacketSize = 1200;
static double g_burstOnTime = 0.2;  // seconds
static double g_burstOffTime = 0.3; // seconds

static double g_backgroundRateMbps = 2.0;
static uint32_t g_backgroundPacketSize = 200;

static NodeContainer g_ueNodes;
static std::vector<Ptr<MobilityModel>> g_ueMobility;
static std::vector<std::vector<Ptr<PacketSink>>> g_ueSinks;
static std::unordered_map<uint16_t, uint32_t> g_rntiToUeIndex;

static bool g_channelGood = true;
static std::string g_scenarioLabel;

static std::vector<Counters> g_ueCounters;
static std::vector<Counters> g_baseCounters;
static std::vector<Counters> g_lastCounters;
static std::vector<uint64_t> g_baseRxBytes;
static std::vector<uint64_t> g_lastRxBytes;
static double g_baseTime = 0.0;
static double g_lastStatsTime = 0.0;

static uint64_t
GetUeRxBytes(uint32_t ueIdx)
{
    uint64_t total = 0;
    if (ueIdx >= g_ueSinks.size())
    {
        return 0;
    }
    for (const auto& sink : g_ueSinks[ueIdx])
    {
        if (sink)
        {
            total += sink->GetTotalRx();
        }
    }
    return total;
}

static void
ForceUeBwp(Ptr<NrUeNetDevice> ueDev,
           Ptr<BwpManagerGnb> gnbMgr,
           uint8_t bwpId,
           uint32_t ueIndex)
{
    Ptr<NrUePhy> uePhy = ueDev->GetPhy(0);
    uint16_t rnti = uePhy->GetRnti();
    if (rnti == 0)
    {
        Simulator::Schedule(MilliSeconds(1), &ForceUeBwp, ueDev, gnbMgr, bwpId, ueIndex);
        return;
    }

    g_rntiToUeIndex[rnti] = ueIndex;
    ueDev->GetBwpManager()->ForceActiveBwp(bwpId);
    gnbMgr->ForceUeBwp(rnti, bwpId);
    NS_LOG_INFO("Forced UE" << ueIndex << " RNTI=" << rnti << " to BWP " << +bwpId);
}

static void
OnRxPacketTraceUe(uint32_t ueIdx, RxPacketTraceParams params)
{
    if (ueIdx >= g_ueCounters.size())
    {
        return;
    }
    g_ueCounters[ueIdx].tbTotal++;
    g_ueCounters[ueIdx].prbTotal += params.m_rbAssignedNum;
    if (params.m_corrupt)
    {
        g_ueCounters[ueIdx].tbError++;
    }
}

static void
OnDlHarqFeedback(const DlHarqInfo& info)
{
    auto it = g_rntiToUeIndex.find(info.m_rnti);
    if (it == g_rntiToUeIndex.end())
    {
        return;
    }
    uint32_t ueIdx = it->second;
    if (ueIdx >= g_ueCounters.size())
    {
        return;
    }
    g_ueCounters[ueIdx].harqTotal++;
    if (info.m_harqStatus == DlHarqInfo::NACK)
    {
        g_ueCounters[ueIdx].harqNack++;
    }
    if (info.m_numRetx > 0)
    {
        g_ueCounters[ueIdx].harqRetx++;
    }
}

static void
PrintPeriodicStats()
{
    double now = Simulator::Now().GetSeconds();
    double interval = now - g_lastStatsTime;
    for (uint32_t ueIdx = 0; ueIdx < g_numUes; ++ueIdx)
    {
        uint64_t totalRx = GetUeRxBytes(ueIdx);
        uint64_t deltaRx = totalRx - g_lastRxBytes[ueIdx];

        Counters delta;
        delta.tbTotal = g_ueCounters[ueIdx].tbTotal - g_lastCounters[ueIdx].tbTotal;
        delta.tbError = g_ueCounters[ueIdx].tbError - g_lastCounters[ueIdx].tbError;
        delta.harqTotal = g_ueCounters[ueIdx].harqTotal - g_lastCounters[ueIdx].harqTotal;
        delta.harqNack = g_ueCounters[ueIdx].harqNack - g_lastCounters[ueIdx].harqNack;
        delta.harqRetx = g_ueCounters[ueIdx].harqRetx - g_lastCounters[ueIdx].harqRetx;
        delta.prbTotal = g_ueCounters[ueIdx].prbTotal - g_lastCounters[ueIdx].prbTotal;

        double throughputMbps = 0.0;
        if (interval > 0.0)
        {
            throughputMbps = (deltaRx * 8.0) / interval / 1e6;
        }
        double bler =
            (delta.tbTotal > 0) ? (static_cast<double>(delta.tbError) / delta.tbTotal) : 0.0;
        double retxRate = (delta.harqTotal > 0)
                              ? (static_cast<double>(delta.harqRetx) / delta.harqTotal)
                              : 0.0;
        double nackRate = (delta.harqTotal > 0)
                              ? (static_cast<double>(delta.harqNack) / delta.harqTotal)
                              : 0.0;
        double prbPerSec = (interval > 0.0) ? (static_cast<double>(delta.prbTotal) / interval) : 0.0;
        double prbPerTb =
            (delta.tbTotal > 0) ? (static_cast<double>(delta.prbTotal) / delta.tbTotal) : 0.0;

        NS_LOG_UNCOND(std::fixed << std::setprecision(3)
                                 << "STAT t=" << now << "s"
                                 << " ue=" << ueIdx
                                 << " scenario=" << g_scenarioLabel
                                 << " channel=" << (g_channelGood ? "GOOD" : "BAD")
                                 << " thr=" << throughputMbps << " Mbps"
                                 << " bler=" << bler
                                 << " retxRate=" << retxRate
                                 << " nackRate=" << nackRate
                                 << " prbSum=" << delta.prbTotal
                                 << " prbPerSec=" << prbPerSec
                                 << " prbPerTb=" << prbPerTb);
        g_lastCounters[ueIdx] = g_ueCounters[ueIdx];
        g_lastRxBytes[ueIdx] = totalRx;
    }

    g_lastStatsTime = now;

    Simulator::Schedule(Seconds(g_statsPeriod), &PrintPeriodicStats);
}
static void
ApplyChannelState(bool good)
{
    g_channelGood = good;
    double baseDist = good ? g_goodDistance : g_badDistance;
    for (uint32_t i = 0; i < g_ueMobility.size(); ++i)
    {
        if (!g_ueMobility[i])
        {
            continue;
        }
        double y = baseDist + static_cast<double>(i);
        g_ueMobility[i]->SetPosition(Vector(0.0, y, g_ueHeight));
    }
    NS_LOG_UNCOND(Simulator::Now().As(Time::MS) << ": CHANNEL "
                                                << (good ? "GOOD" : "BAD")
                                                << " dist=" << baseDist << "m");
}

static void
ToggleChannel()
{
    ApplyChannelState(!g_channelGood);
    if (g_channelPeriod > 0.0)
    {
        Simulator::Schedule(Seconds(g_channelPeriod), &ToggleChannel);
    }
}

static void
InitializeStatsWindow()
{
    g_baseTime = Simulator::Now().GetSeconds();
    g_baseCounters = g_ueCounters;
    g_lastCounters = g_ueCounters;
    g_baseRxBytes.assign(g_numUes, 0);
    g_lastRxBytes.assign(g_numUes, 0);
    for (uint32_t ueIdx = 0; ueIdx < g_numUes; ++ueIdx)
    {
        uint64_t totalRx = GetUeRxBytes(ueIdx);
        g_baseRxBytes[ueIdx] = totalRx;
        g_lastRxBytes[ueIdx] = totalRx;
    }
    g_lastStatsTime = g_baseTime;
    Simulator::Schedule(Seconds(g_statsPeriod), &PrintPeriodicStats);
}

static std::string
RateToStringMbps(double mbps)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << mbps << "Mbps";
    return oss.str();
}

static void
RunScenario()
{
    g_scenarioLabel = "dual-bwp";
    g_ueCounters.assign(g_numUes, Counters{});
    g_baseCounters.assign(g_numUes, Counters{});
    g_lastCounters.assign(g_numUes, Counters{});
    g_baseRxBytes.assign(g_numUes, 0);
    g_lastRxBytes.assign(g_numUes, 0);
    g_baseTime = 0.0;
    g_lastStatsTime = 0.0;

    g_ueSinks.clear();
    g_ueSinks.resize(g_numUes);
    g_ueMobility.clear();
    g_rntiToUeIndex.clear();

    NS_LOG_UNCOND("=== Scenario dual-bwp ===");
    NS_LOG_UNCOND("lowBand=" << std::fixed << std::setprecision(3) << (g_lowFreqHz / 1e9)
                             << "GHz bw=" << std::fixed << std::setprecision(1)
                             << (g_lowBwHz / 1e6) << "MHz mcs=" << +g_midMcs);
    NS_LOG_UNCOND("highBand=" << std::fixed << std::setprecision(3) << (g_highFreqHz / 1e9)
                              << "GHz bw=" << std::fixed << std::setprecision(1)
                              << (g_highBwHz / 1e6) << "MHz mcs=" << +g_lowMcs);

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    g_ueNodes.Create(g_numUes);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
    gnbPos->Add(Vector(0.0, 0.0, g_gnbHeight));
    mobility.SetPositionAllocator(gnbPos);
    mobility.Install(gnbNodes);

    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    double initDist = g_startGood ? g_goodDistance : g_badDistance;
    for (uint32_t i = 0; i < g_numUes; ++i)
    {
        uePos->Add(Vector(0.0, initDist + static_cast<double>(i), g_ueHeight));
    }
    mobility.SetPositionAllocator(uePos);
    mobility.Install(g_ueNodes);

    g_ueMobility.reserve(g_numUes);
    for (uint32_t i = 0; i < g_numUes; ++i)
    {
        g_ueMobility.push_back(g_ueNodes.Get(i)->GetObject<MobilityModel>());
    }

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();

    nrHelper->SetBeamformingHelper(beamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);
    beamformingHelper->SetAttribute("BeamformingMethod",
                                    TypeIdValue(DirectPathBeamforming::GetTypeId()));

    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf lowConf(g_lowFreqHz, g_lowBwHz, 1);
    CcBwpCreator::SimpleOperationBandConf highConf(g_highFreqHz, g_highBwHz, 1);
    OperationBandInfo lowBand = ccBwpCreator.CreateOperationBandContiguousCc(lowConf);
    OperationBandInfo highBand = ccBwpCreator.CreateOperationBandContiguousCc(highConf);
    channelHelper->AssignChannelsToBands({lowBand, highBand});
    auto allBwps = CcBwpCreator::GetAllBwps({lowBand, highBand});

    nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaRR::GetTypeId());
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(true));

    Config::SetDefault("ns3::NrAmc::ErrorModelType", TypeIdValue(TypeId::LookupByName(g_errorModel)));
    nrHelper->SetDlErrorModel(g_errorModel);
    nrHelper->SetUlErrorModel(g_errorModel);

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(g_gnbTxPowerDbm));

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(g_ueNodes, allBwps);

    InternetStackHelper internet;
    internet.Install(g_ueNodes);

    auto [remoteHost, remoteHostAddr] =
        epcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.000));
    (void)remoteHostAddr;

    Ipv4InterfaceContainer ueIfaces = epcHelper->AssignUeIpv4Address(ueDevs);

    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    // Configure per-BWP MCS and connect HARQ feedback trace at gNB MAC
    Ptr<NrGnbNetDevice> gnbNetDev = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(0));
    Ptr<BwpManagerGnb> gnbBwpMgr = gnbNetDev->GetBwpManager();
    if (gnbBwpMgr)
    {
        gnbBwpMgr->SetAttributeSwitchingDelay(MilliSeconds(0));
    }
    for (uint8_t bwpId = 0; bwpId < gnbNetDev->GetCcMapSize(); ++bwpId)
    {
        Ptr<NrMacScheduler> sched = gnbNetDev->GetScheduler(bwpId);
        uint8_t mcs = (bwpId == 0) ? g_midMcs : g_lowMcs;
        sched->SetAttribute("FixedMcsDl", BooleanValue(true));
        sched->SetAttribute("FixedMcsUl", BooleanValue(true));
        sched->SetAttribute("StartingMcsDl", UintegerValue(mcs));
        sched->SetAttribute("StartingMcsUl", UintegerValue(mcs));
    }

    Ptr<NrGnbMac> gnbMac = gnbNetDev->GetMac(0);
    gnbMac->TraceConnectWithoutContext("DlHarqFeedback", MakeCallback(&OnDlHarqFeedback));

    // Connect UE PHY Rx trace for BLER/PRB on all BWPs
    for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(ueDevs.Get(i));
        for (uint8_t bwpId = 0; bwpId < ueDev->GetCcMapSize(); ++bwpId)
        {
            Ptr<NrSpectrumPhy> ueSpectrumPhy = ueDev->GetPhy(bwpId)->GetSpectrumPhy();
            ueSpectrumPhy->TraceConnectWithoutContext("RxPacketTraceUe",
                                                      MakeBoundCallback(&OnRxPacketTraceUe, i));
        }
    }

    if (!gnbBwpMgr)
    {
        NS_LOG_UNCOND("Failed to get BwpManagerGnb");
    }
    else
    {
        for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
        {
            Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(ueDevs.Get(i));
            uint8_t targetBwp = (i == 0) ? 0 : 1;
            ueDev->GetBwpManager()->SetAttributeSwitchingDelay(MilliSeconds(0));
            ForceUeBwp(ueDev, gnbBwpMgr, targetBwp, i);
        }
    }
    // Applications: background + bursty DL
    uint16_t basePort = 5000;
    ApplicationContainer serverApps;
    ApplicationContainer clientApps;

    for (uint32_t i = 0; i < g_numUes; ++i)
    {
        uint16_t burstPort = basePort + i * 2;
        uint16_t bgPort = burstPort + 1;

        PacketSinkHelper burstSinkHelper("ns3::UdpSocketFactory",
                                         InetSocketAddress(Ipv4Address::GetAny(), burstPort));
        ApplicationContainer burstSinkApp = burstSinkHelper.Install(g_ueNodes.Get(i));
        serverApps.Add(burstSinkApp);
        g_ueSinks[i].push_back(DynamicCast<PacketSink>(burstSinkApp.Get(0)));

        PacketSinkHelper bgSinkHelper("ns3::UdpSocketFactory",
                                      InetSocketAddress(Ipv4Address::GetAny(), bgPort));
        ApplicationContainer bgSinkApp = bgSinkHelper.Install(g_ueNodes.Get(i));
        serverApps.Add(bgSinkApp);
        g_ueSinks[i].push_back(DynamicCast<PacketSink>(bgSinkApp.Get(0)));

        OnOffHelper burstClient("ns3::UdpSocketFactory",
                                InetSocketAddress(ueIfaces.GetAddress(i), burstPort));
        burstClient.SetAttribute("PacketSize", UintegerValue(g_burstPacketSize));
        burstClient.SetAttribute("DataRate", DataRateValue(DataRate(RateToStringMbps(g_burstRateMbps))));
        burstClient.SetAttribute("OnTime",
                                 StringValue("ns3::ConstantRandomVariable[Constant=" +
                                             std::to_string(g_burstOnTime) + "]"));
        burstClient.SetAttribute("OffTime",
                                 StringValue("ns3::ConstantRandomVariable[Constant=" +
                                             std::to_string(g_burstOffTime) + "]"));
        clientApps.Add(burstClient.Install(remoteHost));

        UdpClientHelper bgClient(ueIfaces.GetAddress(i), bgPort);
        bgClient.SetAttribute("PacketSize", UintegerValue(g_backgroundPacketSize));
        bgClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        if (g_backgroundRateMbps > 0.0)
        {
            double intervalSec = (g_backgroundPacketSize * 8.0) / (g_backgroundRateMbps * 1e6);
            if (intervalSec <= 0.0)
            {
                intervalSec = 1e-6;
            }
            bgClient.SetAttribute("Interval", TimeValue(Seconds(intervalSec)));
        }
        clientApps.Add(bgClient.Install(remoteHost));
    }

    serverApps.Start(Seconds(g_appStart));
    serverApps.Stop(Seconds(g_simTime));
    clientApps.Start(Seconds(g_appStart));
    clientApps.Stop(Seconds(g_simTime));

    // Initialize channel state and schedule toggling
    g_channelGood = g_startGood;
    ApplyChannelState(g_channelGood);
    if (g_channelPeriod > 0.0)
    {
        Simulator::Schedule(Seconds(g_channelPeriod), &ToggleChannel);
    }

    // Start periodic stats after apps start
    Simulator::Schedule(Seconds(g_appStart), &InitializeStatsWindow);

    Simulator::Stop(Seconds(g_simTime));
    Simulator::Run();

    // Final summary (per UE)
    double duration = g_simTime - g_baseTime;
    NS_LOG_UNCOND("=== Summary dual-bwp ===");
    for (uint32_t ueIdx = 0; ueIdx < g_numUes; ++ueIdx)
    {
        uint64_t totalRx = GetUeRxBytes(ueIdx);
        uint64_t deltaRx =
            (totalRx >= g_baseRxBytes[ueIdx]) ? (totalRx - g_baseRxBytes[ueIdx]) : 0;

        Counters delta;
        delta.tbTotal = g_ueCounters[ueIdx].tbTotal - g_baseCounters[ueIdx].tbTotal;
        delta.tbError = g_ueCounters[ueIdx].tbError - g_baseCounters[ueIdx].tbError;
        delta.harqTotal = g_ueCounters[ueIdx].harqTotal - g_baseCounters[ueIdx].harqTotal;
        delta.harqNack = g_ueCounters[ueIdx].harqNack - g_baseCounters[ueIdx].harqNack;
        delta.harqRetx = g_ueCounters[ueIdx].harqRetx - g_baseCounters[ueIdx].harqRetx;
        delta.prbTotal = g_ueCounters[ueIdx].prbTotal - g_baseCounters[ueIdx].prbTotal;

        double avgThr = (duration > 0.0) ? (deltaRx * 8.0) / duration / 1e6 : 0.0;
        double bler =
            (delta.tbTotal > 0) ? (static_cast<double>(delta.tbError) / delta.tbTotal) : 0.0;
        double retxRate =
            (delta.harqTotal > 0) ? (static_cast<double>(delta.harqRetx) / delta.harqTotal)
                                  : 0.0;
        double nackRate =
            (delta.harqTotal > 0) ? (static_cast<double>(delta.harqNack) / delta.harqTotal)
                                  : 0.0;
        double prbPerSec =
            (duration > 0.0) ? (static_cast<double>(delta.prbTotal) / duration) : 0.0;
        double prbPerTb =
            (delta.tbTotal > 0) ? (static_cast<double>(delta.prbTotal) / delta.tbTotal) : 0.0;

        std::string bandLabel = (ueIdx == 0) ? "lowBand" : "highBand";
        NS_LOG_UNCOND("UE" << ueIdx << " (" << bandLabel << ") avgThroughput="
                           << std::fixed << std::setprecision(3) << avgThr << " Mbps"
                           << " bler=" << bler << " retxRate=" << retxRate
                           << " nackRate=" << nackRate
                           << " prbSum=" << delta.prbTotal
                           << " prbPerSec=" << prbPerSec
                           << " prbPerTb=" << prbPerTb);
    }

    Simulator::Destroy();
}
int
main(int argc, char* argv[])
{
    CommandLine cmd(__FILE__);
    cmd.AddValue("numUes", "Number of UEs", g_numUes);
    cmd.AddValue("simTime", "Simulation time (s)", g_simTime);
    cmd.AddValue("appStart", "Application start time (s)", g_appStart);
    cmd.AddValue("statsPeriod", "Periodic stats interval (s)", g_statsPeriod);
    cmd.AddValue("channelPeriod", "Channel toggle period (s)", g_channelPeriod);
    cmd.AddValue("startGood", "Start in good channel state", g_startGood);
    cmd.AddValue("goodDistance", "Good channel distance (m)", g_goodDistance);
    cmd.AddValue("badDistance", "Bad channel distance (m)", g_badDistance);
    cmd.AddValue("gnbTxPowerDbm", "gNB Tx power (dBm)", g_gnbTxPowerDbm);

    cmd.AddValue("highFreqHz", "High band frequency (Hz)", g_highFreqHz);
    cmd.AddValue("highBwHz", "High band bandwidth (Hz)", g_highBwHz);
    cmd.AddValue("lowFreqHz", "Low band frequency (Hz)", g_lowFreqHz);
    cmd.AddValue("lowBwHz", "Low band bandwidth (Hz)", g_lowBwHz);
    cmd.AddValue("lowMcs", "Low MCS for high band", g_lowMcs);
    cmd.AddValue("midMcs", "Mid MCS for low band", g_midMcs);
    cmd.AddValue("errorModel", "NR error model", g_errorModel);

    cmd.AddValue("burstRateMbps", "Bursty on-interval data rate (Mbps)", g_burstRateMbps);
    cmd.AddValue("burstPacketSize", "Bursty packet size (bytes)", g_burstPacketSize);
    cmd.AddValue("burstOnTime", "Bursty on-time (s)", g_burstOnTime);
    cmd.AddValue("burstOffTime", "Bursty off-time (s)", g_burstOffTime);

    cmd.AddValue("backgroundRateMbps", "Background rate (Mbps)", g_backgroundRateMbps);
    cmd.AddValue("backgroundPacketSize", "Background packet size (bytes)", g_backgroundPacketSize);
    cmd.Parse(argc, argv);

    if (g_numUes != 2)
    {
        NS_LOG_UNCOND("This scenario expects numUes=2; overriding to 2.");
        g_numUes = 2;
    }

    Config::SetDefault("ns3::NrGnbMac::EnableRlcIatKfLog", BooleanValue(false));
    Config::SetDefault("ns3::NrGnbMac::EnableRlcBytesKfLog", BooleanValue(false));

    RunScenario();

    return 0;
}
