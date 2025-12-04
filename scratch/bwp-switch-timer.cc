// A minimal NR scenario (1 gNB, 1 UE, 1 CC with 2 BWPs) that triggers a BWP switch via a
// timer-injected DL DCI. Built from cttc-nr-cc-bwp-demo.cc patterns.

#include "ns3/applications-module.h"
#include "ns3/bwp-manager-ue.h"
#include "ns3/cc-bwp-helper.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-control-messages.h"
#include "ns3/nr-epc-helper.h"
#include "ns3/nr-epc-tft.h"
#include "ns3/eps-bearer.h"
#include "ns3/nr-gnb-mac.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/bwp-manager-gnb.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BwpSwitchTimerExample");

static void
ReportActiveBwp(Ptr<BwpManagerUe> bwpMgr, const std::string& tag)
{
    NS_LOG_UNCOND(tag << " activeBwp=" << +bwpMgr->GetActiveBwp() << " at "
                      << Simulator::Now().GetSeconds() << "s");
}

int
main(int argc, char* argv[])
{
    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnable("BwpSwitchTimerExample", LOG_LEVEL_INFO);
    LogComponentEnable("BwpManagerUe", LOG_LEVEL_INFO);
    // LogComponentEnable("NrUePhy", LOG_LEVEL_DEBUG);
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

    // Build band with 1 CC, 2 BWPs
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf;
    bandConf.m_centralFrequency = 3.5e9;
    bandConf.m_channelBandwidth = 20e6;
    bandConf.m_numCc = 1;
    bandConf.m_numBwp = 2;
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // Channel helper assigns propagation/channel models
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->AssignChannelsToBands({band});
    auto allBwps = CcBwpCreator::GetAllBwps({band});

    // Install NR devices
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Simple EPC/Internet setup to generate DL traffic
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
    (void)internetIpIfaces; // unused

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

    UdpClientHelper dlClient(ueIpIface.GetAddress(0), dlPort);
    dlClient.SetAttribute("PacketSize", UintegerValue(200));
    dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
    dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    ApplicationContainer clientApps = dlClient.Install(remoteHost.Get(0));

    Ptr<NrEpcTft> tft = Create<NrEpcTft>();
    NrEpcTft::PacketFilter dlpf;
    dlpf.localPortStart = dlPort;
    dlpf.localPortEnd = dlPort;
    tft->Add(dlpf);
    NrEpsBearer bearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
    nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(0), bearer, tft);

    Ptr<NrUeNetDevice> ue = ueDevs.Get(0)->GetObject<NrUeNetDevice>();
    Ptr<BwpManagerUe> bwpMgr = ue->GetBwpManager();
    Ptr<NrUePhy> uePhy0 = ue->GetPhy(0);

    // Force initial BWP to 0 and report
    bwpMgr->ForceActiveBwp(0);
    Simulator::Schedule(MilliSeconds(0.5),
                        &ReportActiveBwp,
                        bwpMgr,
                        std::string("Before switch"));

    // Use MAC mapping to switch to BWP1 at 2 ms, and back to BWP0 at 200 ms
    Ptr<NrGnbNetDevice> gnb = gnbDevs.Get(0)->GetObject<NrGnbNetDevice>();
    Ptr<BwpManagerGnb> gnbBwpMgr =
        DynamicCast<BwpManagerGnb>(gnb->GetComponentCarrierManager());
    // RNTI may not be assigned at t=0; poll until nonzero then force.
    std::function<void(Time, uint8_t)> forceToBwp = [&](Time delay, uint8_t targetBwp) {
        Simulator::Schedule(delay, [&, targetBwp]() {
            uint16_t rnti = uePhy0->GetRnti();
            if (rnti == 0)
            {
                // Re-schedule until RNTI is assigned
                forceToBwp(MilliSeconds(1), targetBwp);
            }
            else
            {
                bwpMgr->ForceActiveBwp(targetBwp);
                gnbBwpMgr->ForceUeBwp(rnti, targetBwp);
            }
        });
    };

    forceToBwp(MilliSeconds(2.0), 1);

    Simulator::Schedule(MilliSeconds(100.0),
                        &ReportActiveBwp,
                        bwpMgr,
                        std::string("After switch to 1"));

    forceToBwp(MilliSeconds(200.0), 0);

    Simulator::Schedule(MilliSeconds(500.0),
                        &ReportActiveBwp,
                        bwpMgr,
                        std::string("After switch back to 0"));

    serverApps.Start(MilliSeconds(0.1));
    clientApps.Start(MilliSeconds(0.1));
    serverApps.Stop(Seconds(1));
    clientApps.Stop(Seconds(1));

    Simulator::Stop(Seconds(1));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
