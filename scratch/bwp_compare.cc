// Compare throughput-vs-distance behavior for fixed BWP0 vs fixed BWP1
// in a full-buffer single-UE NR scenario.
//
// Notes:
// - BWP configuration mirrors scratch/aoi-prb-urban-appmix.cc
//   * BWP0: 3.5 GHz, 20 MHz
//   * BWP1: 6.0 GHz, 80 MHz
// - CQI->MCS mapping helper is copied from aoi-prb-urban-appmix.cc.
// - This script runs two simulations sequentially (fixed BWP0, fixed BWP1)
//   and appends samples to one CSV.

#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/bwp-manager-gnb.h"
#include "ns3/config.h"
#include "ns3/core-module.h"
#include "ns3/ideal-beamforming-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-scheduler-ofdma-rr.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/bwp-manager-ue.h"
#include "ns3/bwp-manager-gnb.h"
#include "ns3/point-to-point-helper.h"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BwpCompare");

struct WindowStats
{
    double cqiSum{0.0};
    uint64_t cqiCount{0};
    double mcsSum{0.0};
    uint64_t mcsCount{0};
};

struct RunContext
{
    uint8_t bwpId{0};
    Ptr<PacketSink> sink;
    Ptr<MobilityModel> gnbMob;
    Ptr<MobilityModel> ueMob;
    double samplePeriodS{0.5};
    double simTimeS{20.0};
    uint64_t lastRxBytes{0};
    WindowStats win;
    std::ofstream* out{nullptr};
};

static RunContext g_ctx;

// Same CQI->MCS baseline mapping used in aoi-prb-urban-appmix.cc
static int32_t
EstimateBaseMcsFromCqi(double cqi)
{
    if (!std::isfinite(cqi) || cqi <= 0.0)
    {
        return 0;
    }

    const double cqiClamped = std::max(1.0, std::min(15.0, cqi));
    const double normalized = (cqiClamped - 1.0) / 14.0;
    return static_cast<int32_t>(std::round(normalized * 27.0));
}

static void
OnRxPacketTraceUe(uint32_t ueIdx, RxPacketTraceParams params)
{
    if (ueIdx != 0)
    {
        return;
    }

    if (params.m_cqi != std::numeric_limits<uint8_t>::max())
    {
        g_ctx.win.cqiSum += static_cast<double>(params.m_cqi);
        g_ctx.win.cqiCount++;
    }

    if (params.m_mcs != std::numeric_limits<uint8_t>::max())
    {
        g_ctx.win.mcsSum += static_cast<double>(params.m_mcs);
        g_ctx.win.mcsCount++;
    }
}

static void
SampleAndLog()
{
    const double now = Simulator::Now().GetSeconds();
    if (now <= 0.0 || !g_ctx.sink || !g_ctx.out || !g_ctx.ueMob || !g_ctx.gnbMob)
    {
        return;
    }

    const uint64_t curRx = g_ctx.sink->GetTotalRx();
    const uint64_t deltaRx = (curRx >= g_ctx.lastRxBytes) ? (curRx - g_ctx.lastRxBytes) : 0;
    g_ctx.lastRxBytes = curRx;

    const double thrMbps = (static_cast<double>(deltaRx) * 8.0) / std::max(1e-9, g_ctx.samplePeriodS) / 1e6;

    const Vector gnbPos = g_ctx.gnbMob->GetPosition();
    const Vector uePos = g_ctx.ueMob->GetPosition();
    const double dx = uePos.x - gnbPos.x;
    const double dy = uePos.y - gnbPos.y;
    const double dz = uePos.z - gnbPos.z;
    const double distM = std::sqrt(dx * dx + dy * dy + dz * dz);

    const double avgCqi =
        (g_ctx.win.cqiCount > 0) ? (g_ctx.win.cqiSum / static_cast<double>(g_ctx.win.cqiCount)) : -1.0;
    const double avgMcs =
        (g_ctx.win.mcsCount > 0) ? (g_ctx.win.mcsSum / static_cast<double>(g_ctx.win.mcsCount)) : -1.0;
    const int32_t mappedMcs = EstimateBaseMcsFromCqi(avgCqi);

    (*g_ctx.out) << std::fixed << std::setprecision(6)
                 << static_cast<uint32_t>(g_ctx.bwpId) << ","
                 << now << ","
                 << distM << ","
                 << thrMbps << ","
                 << avgCqi << ","
                 << avgMcs << ","
                 << mappedMcs << "\n";

    g_ctx.win = WindowStats{};

    const double next = now + g_ctx.samplePeriodS;
    if (next < g_ctx.simTimeS - 1e-9)
    {
        Simulator::Schedule(Seconds(g_ctx.samplePeriodS), &SampleAndLog);
    }
}

static void
ForceFixedBwpWithRetry(Ptr<NrUeNetDevice> ueDev, Ptr<BwpManagerGnb> gnbBwpMgr, uint8_t targetBwp)
{
    if (!ueDev)
    {
        return;
    }

    Ptr<NrUePhy> uePhy = ueDev->GetPhy(0);
    if (!uePhy)
    {
        Simulator::Schedule(MilliSeconds(1), &ForceFixedBwpWithRetry, ueDev, gnbBwpMgr, targetBwp);
        return;
    }

    const uint16_t rnti = uePhy->GetRnti();
    if (rnti == 0)
    {
        Simulator::Schedule(MilliSeconds(1), &ForceFixedBwpWithRetry, ueDev, gnbBwpMgr, targetBwp);
        return;
    }

    Ptr<BwpManagerUe> ueMgr = ueDev->GetBwpManager();
    if (ueMgr)
    {
        ueMgr->ForceActiveBwp(targetBwp);
    }
    if (gnbBwpMgr)
    {
        gnbBwpMgr->ForceUeBwp(rnti, targetBwp);
    }
}

static void
RunSingleBwpCase(uint8_t fixedBwpId,
                 double simTime,
                 double appStart,
                 double samplePeriod,
                 double startDistance,
                 double ueSpeed,
                 std::ofstream& out)
{
    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.Install(gnbNodes);
    gnbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 10.0));

    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobility.Install(ueNodes);
    Ptr<ConstantVelocityMobilityModel> ueCv = ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
    ueCv->SetPosition(Vector(startDistance, 0.0, 1.5));
    ueCv->SetVelocity(Vector(std::abs(ueSpeed), 0.0, 0.0));

    // Add one blocker building between gNB (x=0) and UE path (+x direction).
    {
        Ptr<Building> b = CreateObject<Building>();
        b->SetBoundaries(Box(4.0, 8.0, -4.0, 4.0, 0.0, 20.0));
        b->SetBuildingType(Building::Residential);
        b->SetExtWallsType(Building::ConcreteWithWindows);
        b->SetNFloors(4);
        b->SetNRoomsX(5);
        b->SetNRoomsY(5);
    }

    BuildingsHelper::Install(gnbNodes);
    BuildingsHelper::Install(ueNodes);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
    beamformingHelper->SetAttribute("BeamformingPeriodicity", TimeValue(MilliSeconds(1000)));

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(beamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaRR::GetTypeId());
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(false));
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(false));

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Buildings", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(50.0)));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf lowConf(3.5e9, 20e6, 1);
    CcBwpCreator::SimpleOperationBandConf highConf(6.0e9, 80e6, 1);
    OperationBandInfo bandLow = ccBwpCreator.CreateOperationBandContiguousCc(lowConf);
    OperationBandInfo bandHigh = ccBwpCreator.CreateOperationBandContiguousCc(highConf);
    channelHelper->AssignChannelsToBands({bandLow, bandHigh});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({bandLow, bandHigh});

    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(10));
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    InternetStackHelper internet;
    internet.Install(ueNodes);

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    Ptr<Node> remoteHost = CreateObject<Node>();
    internet.Install(remoteHost);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    NetDeviceContainer ueNetDev = ueDevs;
    NetDeviceContainer gnbNetDev = gnbDevs;

    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);

    Ptr<NrGnbNetDevice> gnbNrDev = DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(0));
    Ptr<NrUeNetDevice> ueNrDev = DynamicCast<NrUeNetDevice>(ueNetDev.Get(0));
    Ptr<BwpManagerGnb> gnbBwpMgr = gnbNrDev ? gnbNrDev->GetBwpManager() : nullptr;

    // Force fixed BWP for this run (both UE and gNB-side mapping)
    Simulator::Schedule(MilliSeconds(10), &ForceFixedBwpWithRetry, ueNrDev, gnbBwpMgr, fixedBwpId);

    const uint16_t dlPort = 5000;
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    ApplicationContainer sinkApp = sinkHelper.Install(ueNodes.Get(0));
    sinkApp.Start(Seconds(appStart * 0.5));
    sinkApp.Stop(Seconds(simTime));

    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(ueIpIface.GetAddress(0), dlPort));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate("5Gb/s")));
    onoff.SetAttribute("PacketSize", UintegerValue(1200));
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer srcApp = onoff.Install(remoteHost);
    srcApp.Start(Seconds(appStart));
    srcApp.Stop(Seconds(simTime));

    // CQI/MCS observation from UE Rx trace
    for (uint8_t bwpId = 0; bwpId < ueNrDev->GetCcMapSize(); ++bwpId)
    {
        Ptr<NrSpectrumPhy> ueSpectrumPhy = ueNrDev->GetPhy(bwpId)->GetSpectrumPhy();
        ueSpectrumPhy->TraceConnectWithoutContext("RxPacketTraceUe", MakeBoundCallback(&OnRxPacketTraceUe, 0));
    }

    g_ctx = RunContext{};
    g_ctx.bwpId = fixedBwpId;
    g_ctx.sink = DynamicCast<PacketSink>(sinkApp.Get(0));
    g_ctx.gnbMob = gnbNodes.Get(0)->GetObject<MobilityModel>();
    g_ctx.ueMob = ueNodes.Get(0)->GetObject<MobilityModel>();
    g_ctx.samplePeriodS = samplePeriod;
    g_ctx.simTimeS = simTime;
    g_ctx.lastRxBytes = 0;
    g_ctx.out = &out;

    Simulator::Schedule(Seconds(appStart + samplePeriod), &SampleAndLog);
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();
}

int
main(int argc, char* argv[])
{
    uint32_t randomSeed = 2;
    uint32_t randomRun = 2;
    double simTime = 20.0;
    double appStart = 0.05;
    double samplePeriod = 0.5;
    double startDistanceM = 0.0;
    double ueSpeedMps = 10.0;
    std::string outCsv = "scratch/rl_bwp/runs/bwp_compare.csv";

    CommandLine cmd(__FILE__);
    cmd.AddValue("randomSeed", "Global ns-3 RNG seed", randomSeed);
    cmd.AddValue("randomRun", "Global ns-3 RNG run", randomRun);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("appStart", "Application start time (s)", appStart);
    cmd.AddValue("samplePeriod", "Sampling period for throughput/CQI logging (s)", samplePeriod);
    cmd.AddValue("startDistanceM", "UE initial distance from gNB (m)", startDistanceM);
    cmd.AddValue("ueSpeedMps", "UE speed moving away from gNB along +x (m/s)", ueSpeedMps);
    cmd.AddValue("outCsv", "Output CSV path", outCsv);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(randomSeed);
    RngSeedManager::SetRun(randomRun);

    std::ofstream out(outCsv, std::ios::out);
    if (!out.is_open())
    {
        NS_ABORT_MSG("Failed to open outCsv: " << outCsv);
    }

    out << "bwp_id,time_s,distance_m,throughput_mbps,avg_cqi,avg_mcs,cqi_mapped_mcs\n";

    // Fixed BWP0 run
    RunSingleBwpCase(0, simTime, appStart, samplePeriod, startDistanceM, ueSpeedMps, out);

    // Fixed BWP1 run
    RunSingleBwpCase(1, simTime, appStart, samplePeriod, startDistanceM, ueSpeedMps, out);

    out.flush();
    out.close();

    NS_LOG_UNCOND("Saved CSV: " << outCsv);
    return 0;
}
