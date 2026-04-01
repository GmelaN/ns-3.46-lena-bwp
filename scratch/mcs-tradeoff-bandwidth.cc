// Scenario: DL-only NR with per-UE fixed MCS via per-BWP schedulers and forced BWP selection.
// Purpose: quantify MCS/channel tradeoff with good/bad distances and per-UE results.

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-module.h"
#include "ns3/nr-mac-scheduler-ofdma-rr.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/cc-bwp-helper.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/bwp-manager-gnb.h"
#include "ns3/bwp-manager-ue.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("McsTradeoffBwp");

static uint32_t g_numUes = 4;
static double g_simTime = 1.0;
static double g_appStart = 0.2;
static uint32_t g_packetSize = 1500;
static double g_intervalMs = 1;
static double g_offeredLoadMbps = 0.0;

static double g_goodDistance = 20.0;
static double g_badDistance = 250.0;
static double g_distanceAll = -1.0;
static double g_distanceStart = 20.0;
static double g_distanceStep = 20.0;

static uint8_t g_mcsBwp0 = 4;
static uint8_t g_mcsBwp1 = 24;

static double g_gnbTxPowerDbm = 20.0;

static double g_bwpFreq = 3.50e9;
static double g_bwp0BwHz = 20e6;
static double g_bwp1BwHz = 20e6;

static std::string g_ueBwpMap;
static std::string g_ueDistances;
static std::string g_mcsLevels = "4,12,20";
static std::string g_bwpSweep = "both";
static std::string g_errorModel = "ns3::NrEesmCcT1";
static std::string g_experiment = "custom";

struct UeFlowStats
{
    uint64_t txPackets{0};
    uint64_t rxPackets{0};
    uint64_t rxBytes{0};
    double delaySumSec{0.0};
    bool hasTx{false};
    bool hasRx{false};
    Time timeFirstTx{Seconds(0)};
    Time timeLastRx{Seconds(0)};
};

static std::string
Trim(const std::string& s)
{
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start])))
    {
        ++start;
    }
    size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1])))
    {
        --end;
    }
    return s.substr(start, end - start);
}

static std::string
ToLower(const std::string& s)
{
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return out;
}

static std::vector<uint8_t>
ParseUint8List(const std::string& s)
{
    std::vector<uint8_t> out;
    if (s.empty())
    {
        return out;
    }
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        item = Trim(item);
        if (item.empty())
        {
            continue;
        }
        int v = 0;
        std::stringstream val(item);
        val >> v;
        if (!val.fail())
        {
            out.push_back(static_cast<uint8_t>(std::max(0, v)));
        }
    }
    return out;
}

static std::vector<double>
ParseDoubleList(const std::string& s)
{
    std::vector<double> out;
    if (s.empty())
    {
        return out;
    }
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        item = Trim(item);
        if (item.empty())
        {
            continue;
        }
        double v = 0.0;
        std::stringstream val(item);
        val >> v;
        if (!val.fail())
        {
            out.push_back(v);
        }
    }
    return out;
}

static std::vector<uint8_t>
BuildBwpMap(uint32_t numUes, const std::vector<uint8_t>& userMap)
{
    std::vector<uint8_t> map(numUes, 0);
    if (!userMap.empty())
    {
        for (uint32_t i = 0; i < numUes; ++i)
        {
            map[i] = userMap[std::min<uint32_t>(i, userMap.size() - 1)];
        }
    }
    else
    {
        for (uint32_t i = 0; i < numUes; ++i)
        {
            map[i] = static_cast<uint8_t>(i % 2);
        }
    }
    for (uint32_t i = 0; i < numUes; ++i)
    {
        map[i] = std::min<uint8_t>(map[i], 1);
    }
    return map;
}

static std::vector<double>
BuildDistanceMap(uint32_t numUes,
                 const std::vector<double>& userDistances,
                 double distanceAll,
                 double distanceStart,
                 double distanceStep)
{
    std::vector<double> map(numUes, distanceStart);
    if (!userDistances.empty())
    {
        for (uint32_t i = 0; i < numUes; ++i)
        {
            map[i] = userDistances[std::min<uint32_t>(i, userDistances.size() - 1)];
        }
    }
    else if (distanceAll >= 0.0)
    {
        for (uint32_t i = 0; i < numUes; ++i)
        {
            map[i] = distanceAll;
        }
    }
    else if (distanceStep > 0.0)
    {
        for (uint32_t i = 0; i < numUes; ++i)
        {
            map[i] = distanceStart + (static_cast<double>(i) * distanceStep);
        }
    }
    else
    {
        uint32_t half = numUes / 2;
        for (uint32_t i = 0; i < numUes; ++i)
        {
            map[i] = (i < half) ? g_goodDistance : g_badDistance;
        }
    }
    return map;
}

static std::vector<uint8_t>
BuildUniformBwpMap(uint32_t numUes, uint8_t bwpId)
{
    return std::vector<uint8_t>(numUes, std::min<uint8_t>(bwpId, 1));
}

static std::vector<uint8_t>
ParseBwpSweep(const std::string& sweep)
{
    std::string v = ToLower(Trim(sweep));
    if (v.empty() || v == "both" || v == "all")
    {
        return {0, 1};
    }
    if (v == "low" || v == "0" || v == "bwp0")
    {
        return {0};
    }
    if (v == "high" || v == "1" || v == "bwp1")
    {
        return {1};
    }
    return {0, 1};
}

static uint8_t
GetMaxMcs(const std::string& errorModel)
{
    std::string v = ToLower(errorModel);
    if (v.find("t2") != std::string::npos)
    {
        return 27;
    }
    return 28;
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

    ueDev->GetBwpManager()->ForceActiveBwp(bwpId);
    gnbMgr->ForceUeBwp(rnti, bwpId);
    NS_LOG_INFO("Forced UE" << ueIndex << " RNTI=" << rnti << " to BWP " << +bwpId);
}

static void
RunScenario(const std::vector<uint8_t>& bwpMap,
            const std::vector<double>& distMap,
            uint8_t mcsBwp0,
            uint8_t mcsBwp1,
            const std::string& runLabel)
{
    if (bwpMap.size() != g_numUes || distMap.size() != g_numUes)
    {
        NS_LOG_UNCOND("BWP map or distance map size does not match numUes");
        return;
    }

    std::cout << "=== Run " << runLabel << " ===" << std::endl;
    std::cout << "errorModel=" << g_errorModel
              << " bwp0=" << std::fixed << std::setprecision(3) << (g_bwpFreq / 1e9) << "GHz"
              << " bwp1=" << std::fixed << std::setprecision(3) << (g_bwpFreq / 1e9) << "GHz"
              << " bw0=" << std::fixed << std::setprecision(1) << (g_bwp0BwHz / 1e6) << "MHz"
              << " bw1=" << std::fixed << std::setprecision(1) << (g_bwp1BwHz / 1e6) << "MHz"
              << " mcs0=" << +mcsBwp0 << " mcs1=" << +mcsBwp1 << std::endl;

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(g_numUes);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
    gnbPos->Add(Vector(0.0, 0.0, 10.0));
    mobility.SetPositionAllocator(gnbPos);
    mobility.Install(gnbNodes);

    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < g_numUes; ++i)
    {
        uePos->Add(Vector(0.0, distMap[i], 1.5));
    }
    mobility.SetPositionAllocator(uePos);
    mobility.Install(ueNodes);

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
    CcBwpCreator::SimpleOperationBandConf bwp0Conf(g_bwpFreq, g_bwp0BwHz, 1);
    CcBwpCreator::SimpleOperationBandConf bwp1Conf(g_bwpFreq, g_bwp1BwHz, 1);
    OperationBandInfo band0 = ccBwpCreator.CreateOperationBandContiguousCc(bwp0Conf);
    OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc(bwp1Conf);
    channelHelper->AssignChannelsToBands({band0, band1});
    auto allBwps = CcBwpCreator::GetAllBwps({band0, band1});

    nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaRR::GetTypeId());
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(false));

    Config::SetDefault("ns3::NrAmc::ErrorModelType",
                       TypeIdValue(TypeId::LookupByName(g_errorModel)));
    nrHelper->SetDlErrorModel(g_errorModel);
    nrHelper->SetUlErrorModel(g_errorModel);

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(g_gnbTxPowerDbm));

    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(0));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(0));

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    Ptr<NrGnbNetDevice> gnbNetDev = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(0));
    Ptr<BwpManagerGnb> gnbBwpMgr =
        DynamicCast<BwpManagerGnb>(gnbNetDev->GetComponentCarrierManager());
    if (!gnbBwpMgr)
    {
        NS_LOG_UNCOND("Failed to get BwpManagerGnb");
        return;
    }
    gnbBwpMgr->SetAttributeSwitchingDelay(MilliSeconds(0));

    for (uint8_t bwpId = 0; bwpId < gnbNetDev->GetCcMapSize(); ++bwpId)
    {
        NrHelper::GetGnbPhy(gnbDevs.Get(0), bwpId)->SetAttribute("TxPower",
                                                                DoubleValue(g_gnbTxPowerDbm));
        Ptr<NrMacScheduler> sched = gnbNetDev->GetScheduler(bwpId);
        uint8_t mcs = (bwpId == 0) ? mcsBwp0 : mcsBwp1;
        sched->SetAttribute("FixedMcsDl", BooleanValue(true));
        sched->SetAttribute("FixedMcsUl", BooleanValue(true));
        sched->SetAttribute("StartingMcsDl", UintegerValue(mcs));
        sched->SetAttribute("StartingMcsUl", UintegerValue(mcs));
    }

    InternetStackHelper internet;
    internet.Install(ueNodes);

    auto [remoteHost, remoteHostAddr] =
        epcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.000));
    static_cast<void>(remoteHostAddr);
    NodeContainer remoteHostContainer(remoteHost);

    Ipv4InterfaceContainer ueIfaces = epcHelper->AssignUeIpv4Address(ueDevs);
    std::unordered_map<Ipv4Address, uint32_t, Ipv4AddressHash> ueIpToIndex;
    for (uint32_t i = 0; i < ueIfaces.GetN(); ++i)
    {
        ueIpToIndex.emplace(ueIfaces.GetAddress(i), i);
    }

    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    for (uint32_t i = 0; i < g_numUes; ++i)
    {
        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(ueDevs.Get(i));
        ueDev->GetBwpManager()->SetAttributeSwitchingDelay(MilliSeconds(0));
        ForceUeBwp(ueDev, gnbBwpMgr, bwpMap[i], i);
    }

    uint16_t basePort = 4000;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;

    for (uint32_t i = 0; i < g_numUes; ++i)
    {
        uint16_t port = basePort + i;
        UdpServerHelper server(port);
        serverApps.Add(server.Install(ueNodes.Get(i)));

        UdpClientHelper client(ueIfaces.GetAddress(i), port);
        client.SetAttribute("PacketSize", UintegerValue(g_packetSize));
        client.SetAttribute("Interval", TimeValue(MilliSeconds(g_intervalMs)));
        client.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        clientApps.Add(client.Install(remoteHostContainer.Get(0)));
    }

    serverApps.Start(Seconds(g_appStart));
    clientApps.Start(Seconds(g_appStart));
    serverApps.Stop(Seconds(g_simTime));
    clientApps.Stop(Seconds(g_simTime));

    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> monitor = flowHelper.InstallAll();

    Simulator::Stop(Seconds(g_simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());

    std::vector<UeFlowStats> stats(g_numUes);
    for (const auto& kv : monitor->GetFlowStats())
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
        auto it = ueIpToIndex.find(t.destinationAddress);
        if (it == ueIpToIndex.end())
        {
            continue;
        }
        uint32_t ueIdx = it->second;
        auto& s = stats[ueIdx];
        s.txPackets += kv.second.txPackets;
        s.rxPackets += kv.second.rxPackets;
        s.rxBytes += kv.second.rxBytes;
        s.delaySumSec += kv.second.delaySum.GetSeconds();
        if (kv.second.txPackets > 0)
        {
            if (!s.hasTx || kv.second.timeFirstTxPacket < s.timeFirstTx)
            {
                s.timeFirstTx = kv.second.timeFirstTxPacket;
                s.hasTx = true;
            }
        }
        if (kv.second.rxPackets > 0)
        {
            if (!s.hasRx || kv.second.timeLastRxPacket > s.timeLastRx)
            {
                s.timeLastRx = kv.second.timeLastRxPacket;
                s.hasRx = true;
            }
        }
    }

    std::cout << "=== UE results (DL) ===" << std::endl;
    for (uint32_t i = 0; i < g_numUes; ++i)
    {
        const auto& s = stats[i];
        uint8_t bwpId = bwpMap[i];
        uint8_t mcs = (bwpId == 0) ? mcsBwp0 : mcsBwp1;
        double bwpBw = (bwpId == 0) ? g_bwp0BwHz : g_bwp1BwHz;
        double bwpFreq = (bwpId == 0) ? g_bwpFreq : g_bwpFreq;
        double dist = distMap[i];
        std::string channelTag = "custom";
        if (std::abs(dist - g_goodDistance) < 1e-6)
        {
            channelTag = "good";
        }
        else if (std::abs(dist - g_badDistance) < 1e-6)
        {
            channelTag = "bad";
        }

        double lossPct = 0.0;
        if (s.txPackets > 0)
        {
            lossPct = (static_cast<double>(s.txPackets - s.rxPackets) / s.txPackets) * 100.0;
        }

        double throughputMbps = 0.0;
        if (s.hasRx && s.hasTx)
        {
            double duration = (s.timeLastRx - s.timeFirstTx).GetSeconds();
            if (duration > 0.0)
            {
                throughputMbps = (s.rxBytes * 8.0) / duration / 1e6;
            }
        }

        double avgDelayMs = 0.0;
        if (s.rxPackets > 0)
        {
            avgDelayMs = (s.delaySumSec / s.rxPackets) * 1000.0;
        }

        std::cout << "UE" << i << " dist=" << dist << "m"
                  << " channel=" << channelTag
                  << " bwp=" << +bwpId
                  << " freq=" << std::fixed << std::setprecision(3) << (bwpFreq / 1e9) << "GHz"
                  << " bw=" << std::fixed << std::setprecision(1) << (bwpBw / 1e6) << "MHz"
                  << " mcs=" << +mcs
                  << " tx=" << s.txPackets
                  << " rx=" << s.rxPackets
                  << " loss=" << std::fixed << std::setprecision(2) << lossPct << "%"
                  << " thr=" << std::fixed << std::setprecision(3) << throughputMbps << " Mbps"
                  << " delay=" << std::fixed << std::setprecision(3) << avgDelayMs << " ms"
                  << std::endl;
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
    cmd.AddValue("packetSize", "UDP packet size (bytes)", g_packetSize);
    cmd.AddValue("intervalMs", "UDP packet interval (ms)", g_intervalMs);
    cmd.AddValue("offeredLoadMbps",
                 "If >0, override interval to match offered load per UE (Mbps)",
                 g_offeredLoadMbps);
    cmd.AddValue("goodDistance", "Distance for good channel (m)", g_goodDistance);
    cmd.AddValue("badDistance", "Distance for bad channel (m)", g_badDistance);
    cmd.AddValue("distance", "If >=0, force all UEs to same distance (m)", g_distanceAll);
    cmd.AddValue("distanceStart", "Start distance for uniform spacing (m)", g_distanceStart);
    cmd.AddValue("distanceStep",
                 "Step distance for uniform spacing (m). Set <=0 to disable",
                 g_distanceStep);
    cmd.AddValue("mcsBwp0", "Fixed MCS for BWP 0 (low)", g_mcsBwp0);
    cmd.AddValue("mcsBwp1", "Fixed MCS for BWP 1 (high)", g_mcsBwp1);
    cmd.AddValue("mcsLevels",
                 "Comma-separated MCS levels to sweep (empty: single run with mcsBwp0/1)",
                 g_mcsLevels);
    cmd.AddValue("gnbTxPowerDbm", "gNB Tx power (dBm)", g_gnbTxPowerDbm);
    cmd.AddValue("bwpFreq", "Center frequency for each BWP (Hz)", g_bwpFreq);
    cmd.AddValue("bwp0BwHz", "Bandwidth for BWP 0 (Hz)", g_bwp0BwHz);
    cmd.AddValue("bwp1BwHz", "Bandwidth for BWP 1 (Hz)", g_bwp1BwHz);
    cmd.AddValue("ueBwpMap", "Comma-separated UE->BWP map (e.g., 0,1,0,1)", g_ueBwpMap);
    cmd.AddValue("ueDistances", "Comma-separated UE distances (m)", g_ueDistances);
    cmd.AddValue("bwpSweep", "low|high|both (used when mcsLevels is set)", g_bwpSweep);
    cmd.AddValue("errorModel", "NR error model", g_errorModel);
    cmd.AddValue("experiment",
                 "custom|mcs|bwp (mcs: same BWP size, different MCS; bwp: same MCS, different BWP)",
                 g_experiment);
    cmd.Parse(argc, argv);

    if (g_numUes == 0)
    {
        NS_LOG_UNCOND("numUes must be >= 1");
        return 1;
    }

    Config::SetDefault("ns3::NrGnbMac::EnableRlcIatKfLog", BooleanValue(false));
    Config::SetDefault("ns3::NrGnbMac::EnableRlcBytesKfLog", BooleanValue(false));

    if (g_offeredLoadMbps > 0.0)
    {
        NS_LOG_UNCOND("Offered Load(Mbps): " << g_offeredLoadMbps);
        double intervalSec = (g_packetSize * 8.0) / (g_offeredLoadMbps * 1e6);
        g_intervalMs = intervalSec * 1000.0;
    }

    if (g_experiment == "mcs")
    {
        g_bwp1BwHz = g_bwp0BwHz;
    }
    else if (g_experiment == "bwp")
    {
        g_mcsBwp1 = g_mcsBwp0;
    }

    const auto userBwpMap = ParseUint8List(g_ueBwpMap);
    const auto userDistances = ParseDoubleList(g_ueDistances);
    const auto distMap =
        BuildDistanceMap(g_numUes, userDistances, g_distanceAll, g_distanceStart, g_distanceStep);

    auto mcsLevels = ParseUint8List(g_mcsLevels);
    uint8_t maxMcs = GetMaxMcs(g_errorModel);
    if (!mcsLevels.empty())
    {
        std::vector<uint8_t> filtered;
        for (auto mcs : mcsLevels)
        {
            if (mcs <= maxMcs)
            {
                filtered.push_back(mcs);
            }
            else
            {
                std::cout << "Skipping MCS " << +mcs << " (max for " << g_errorModel << " is "
                          << +maxMcs << ")" << std::endl;
            }
        }
        mcsLevels.swap(filtered);
    }

    if (mcsLevels.empty())
    {
        const auto bwpMap = BuildBwpMap(g_numUes, userBwpMap);
        RunScenario(bwpMap, distMap, g_mcsBwp0, g_mcsBwp1, "single");
        return 0;
    }

    auto sweepBwps = ParseBwpSweep(g_bwpSweep);
    std::cout << "Sweep mode: bwp=" << g_bwpSweep << " mcsLevels=" << g_mcsLevels
              << " distanceStart=" << g_distanceStart << "m distanceStep=" << g_distanceStep
              << "m numUes=" << g_numUes << std::endl;

    for (auto bwpId : sweepBwps)
    {
        auto bwpMap = BuildUniformBwpMap(g_numUes, bwpId);
        for (auto mcs : mcsLevels)
        {
            std::ostringstream label;
            label << "bwp=" << +bwpId << " mcs=" << +mcs;
            RunScenario(bwpMap, distMap, mcs, mcs, label.str());
        }
    }

    return 0;
}
