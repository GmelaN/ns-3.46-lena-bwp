// Scenario: 1 gNB with two BWPs (narrow/wide), four UEs forced to switch BWPs at different times.
// - UE0: stays on narrow for whole run
// - UE1: narrow until 0.3s, then wide
// - UE2: narrow until 0.7s, then wide
// - UE3: stays on wide for whole run
// Collects per-flow throughput and mean delay using FlowMonitor.

// 지연이 고려된 BWP 스위치 동작 확인 예제 

#include "ns3/applications-module.h"
#include "ns3/bwp-manager-gnb.h"
#include "ns3/bwp-manager-ue.h"
#include "ns3/cc-bwp-helper.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-epc-helper.h"
#include "ns3/nr-epc-tft.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"
#include "ns3/ipv4-flow-classifier.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BwpSwitchMultiExample");

static void ForceAttempt(Ptr<NrUeNetDevice> ueDev,
                         Ptr<BwpManagerGnb> gnbMgr,
                         uint8_t targetBwp);

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
    NS_LOG_UNCOND("Force UE rnti=" << rnti << " to BWP " << +targetBwp << " at "
                                   << Simulator::Now().GetSeconds() << "s");
}

static void
ForceBothSides(Time delay,
               Ptr<NrUeNetDevice> ueDev,
               Ptr<BwpManagerGnb> gnbMgr,
               uint8_t targetBwp)
{
    NS_ABORT_MSG_IF(gnbMgr == nullptr, "BwpManagerGnb not found");
    Simulator::Schedule(delay, &ForceAttempt, ueDev, gnbMgr, targetBwp);
}

int
main(int argc, char* argv[])
{
    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnable("BwpSwitchMultiExample", LOG_LEVEL_INFO);

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(4);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetSchedulerTypeId(NrMacSchedulerTdmaRR::GetTypeId());

    // Two CCs (treated as two BWPs for forcing): narrow 5 MHz and wide 80 MHz.
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf narrowConf(3.50e9, 5e6, 1);
    CcBwpCreator::SimpleOperationBandConf wideConf(3.60e9, 80e6, 1);
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

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2ph.SetChannelAttribute("Delay", StringValue("1ms"));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost.Get(0));
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    (void)internetIpIfaces;

    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(ueDevs);
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

    // Applications: one DL and one UL UDP flow per UE with distinct ports.
    ApplicationContainer serverApps, clientApps;
    ApplicationContainer ulServerApps, ulClientApps;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        uint16_t port = 5000 + i;
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        serverApps.Add(sinkHelper.Install(ueNodes.Get(i)));

        UdpClientHelper client(ueIpIfaces.GetAddress(i), port);
        client.SetAttribute("PacketSize", UintegerValue(1400));
        client.SetAttribute("Interval", TimeValue(MicroSeconds(500)));
        client.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        clientApps.Add(client.Install(remoteHost.Get(0)));

        // UL flow: UE -> remoteHost
        uint16_t ulPort = 6000 + i;
        PacketSinkHelper ulSinkHelper("ns3::UdpSocketFactory",
                                      InetSocketAddress(Ipv4Address::GetAny(), ulPort));
        ulServerApps.Add(ulSinkHelper.Install(remoteHost.Get(0)));
        UdpClientHelper ulClient(internetIpIfaces.GetAddress(1), ulPort);
        ulClient.SetAttribute("PacketSize", UintegerValue(1400));
        ulClient.SetAttribute("Interval", TimeValue(MicroSeconds(500)));
        ulClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        ulClientApps.Add(ulClient.Install(ueNodes.Get(i)));

        Ptr<NrEpcTft> tft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter pf;
        pf.localPortStart = port;
        pf.localPortEnd = port;
        tft->Add(pf);
        NrEpsBearer bearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
        nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(i), bearer, tft);
    }

    serverApps.Start(MilliSeconds(50));
    clientApps.Start(MilliSeconds(50));
    ulServerApps.Start(MilliSeconds(50));
    ulClientApps.Start(MilliSeconds(50));
    serverApps.Stop(Seconds(10));
    clientApps.Stop(Seconds(10));
    ulServerApps.Stop(Seconds(10));
    ulClientApps.Stop(Seconds(10));

    // Initial BWP choices
    std::vector<uint8_t> initialBwp = {1, 1, 1, 1};
    std::vector<std::pair<double, uint8_t>> switches[4];
    // switches[1].push_back({4, 1}); // UE1: narrow -> wide
    // switches[2].push_back({1, 1}); // UE2: narrow -> wide

    switches[0].push_back({1, 1});
    switches[0].push_back({1.2, 0});
    switches[0].push_back({1.5, 1});
    switches[0].push_back({1.8, 0});
    switches[0].push_back({2.1, 1});
    switches[0].push_back({2.5, 0});
    switches[0].push_back({2.8, 1});
    switches[0].push_back({3.2, 0});
    switches[0].push_back({3.6, 1});
    switches[0].push_back({4.1, 0});
    switches[0].push_back({4.5, 1});
    switches[0].push_back({4.8, 0});
    switches[0].push_back({5 + 1, 1});
    switches[0].push_back({5 + 1.2, 0});
    switches[0].push_back({5 + 1.5, 1});
    switches[0].push_back({5 + 1.8, 0});
    switches[0].push_back({5 + 2.1, 1});
    switches[0].push_back({5 + 2.5, 0});
    switches[0].push_back({5 + 2.8, 1});
    switches[0].push_back({5 + 3.2, 0});
    switches[0].push_back({5 + 3.6, 1});
    switches[0].push_back({5 + 4.1, 0});
    switches[0].push_back({5 + 4.5, 1});
    switches[0].push_back({5 + 4.8, 0});
    // UE0 stays narrow; UE3 stays wide.
    std::vector<std::vector<std::pair<double, uint8_t>>> switchTimeline(4);

    for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ueDev = ueDevs.Get(i)->GetObject<NrUeNetDevice>();
        ueDev->GetBwpManager()->ForceActiveBwp(initialBwp[i]);
        ForceBothSides(Seconds(0.0), ueDev, gnbBwpMgr, initialBwp[i]);
        switchTimeline[i].push_back({0.0, initialBwp[i]});
        for (auto sw : switches[i])
        {
            ForceBothSides(Seconds(sw.first), ueDev, gnbBwpMgr, sw.second);
            switchTimeline[i].push_back(sw);
        }
        ueDev->GetBwpManager()->SetAttributeSwitchingDelay(MilliSeconds(20));
    }

    // FlowMonitor for throughput/delay
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    Simulator::Stop(Seconds(10.1));
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
        if (t.destinationPort < 5000)
        {
            // Skip EPC internal control/user-plane tunnels (e.g., 2123/2152).
            continue;
        }
        if (st.rxPackets == 0)
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
        NS_LOG_UNCOND("Flow " << flowId << " dstPort=" << t.destinationPort
                              << " rxPkts=" << st.rxPackets << " throughput=" << throughputMbps
                              << " Mbps meanDelay=" << meanDelayMs << " ms");
    }

    // Simple energy estimate: static power proportional to BWP bandwidth.
    std::vector<double> bwpPowerMw = {100.0, 800.0}; // example mW for narrow/wide
    double simEnd = 5.0;
    for (uint32_t i = 0; i < switchTimeline.size(); ++i)
    {
        double energyJ = 0.0;
        auto timeline = switchTimeline[i];
        timeline.push_back({simEnd, timeline.back().second});
        for (size_t k = 1; k < timeline.size(); ++k)
        {
            double dt = timeline[k].first - timeline[k - 1].first;
            uint8_t bwp = timeline[k - 1].second;
            energyJ += dt * bwpPowerMw[bwp] * 1e-3; // mW * s -> mJ -> J
        }
        NS_LOG_UNCOND("UE" << i << " energy_est=" << energyJ << " J (static BWP power model)");
    }

    Simulator::Destroy();
    return 0;
}
