// Single-gNB, multi-BWP downlink scenario with Poisson traffic.
// - 1 gNB, 10 UEs, 3 BWPs (narrow/mid/wide).
// - No mobility.
// - Downlink Poisson arrivals; packet size can be Poisson or fixed.

#include "ns3/applications-module.h"
#include "ns3/bwp-manager-gnb.h"
#include "ns3/cc-bwp-helper.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-epc-helper.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-scheduler-ofdma-rr.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/nr-bwp-switch-controller.h"

#include <cmath>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrPoissonMultiBwpExample");

static void
RegisterUeManager(const Ptr<NrBwpSwitchController>& ctrl, Ptr<NrUeNetDevice>& ueDev)
{
    uint16_t rnti = ueDev->GetPhy(0)->GetRnti();
    if (rnti == 0)
    {
        Simulator::Schedule(MilliSeconds(1), &RegisterUeManager, ctrl, ueDev);
        return;
    }
    ctrl->AddUeManager(rnti, ueDev->GetBwpManager());
    NS_LOG_INFO("Registered UE rnti=" << rnti << " with BWP switch controller");
}

class PoissonDlApp : public Application
{
  public:
    PoissonDlApp()
    {
        m_interArrival = CreateObject<ExponentialRandomVariable>();
        m_uniform = CreateObject<UniformRandomVariable>();
    }

    void SetPeer(Address peer)
    {
        m_peer = peer;
    }

    void SetMeanInterArrival(double seconds)
    {
        m_interArrival->SetAttribute("Mean", DoubleValue(seconds));
    }

    void SetMeanPacketSize(uint32_t bytes, bool poisson)
    {
        m_meanPkt = bytes;
        m_usePoissonSize = poisson;
    }

    void SetSocket(Ptr<Socket> socket)
    {
        m_socket = socket;
    }

    uint64_t GetPacketsSent() const
    {
        return m_pktsSent;
    }

    uint64_t GetBytesSent() const
    {
        return m_bytesSent;
    }

  private:
    void StartApplication() override
    {
        m_running = true;
        if (m_socket == nullptr)
        {
            m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        }
        m_socket->Connect(m_peer);
        ScheduleNext();
    }

    void StopApplication() override
    {
        m_running = false;
        if (m_event.IsPending())
        {
            Simulator::Cancel(m_event);
        }
        if (m_socket)
        {
            m_socket->Close();
        }
    }

    void ScheduleNext()
    {
        if (!m_running)
        {
            return;
        }
        Time t = Seconds(std::max(1e-6, m_interArrival->GetValue()));
        m_event = Simulator::Schedule(t, &PoissonDlApp::SendPacket, this);
    }

    void SendPacket()
    {
        if (!m_running)
        {
            return;
        }
        uint32_t pktSize = m_usePoissonSize ? std::max<uint32_t>(1, SamplePoisson(m_meanPkt)) : m_meanPkt;
        m_socket->Send(Create<Packet>(pktSize));
        m_pktsSent++;
        m_bytesSent += pktSize;
        // NS_LOG_INFO("PoissonDlApp send size=" << pktSize << " dst=" << InetSocketAddress::ConvertFrom(m_peer).GetIpv4()
        //                                       << ":" << InetSocketAddress::ConvertFrom(m_peer).GetPort()
        //                                       << " t=" << Simulator::Now().GetSeconds());
        ScheduleNext();
    }

    Ptr<Socket> m_socket;
    Address m_peer;
    Ptr<ExponentialRandomVariable> m_interArrival;
    Ptr<UniformRandomVariable> m_uniform;
    uint32_t m_meanPkt{500};
    bool m_usePoissonSize{true};
    bool m_running{false};
    EventId m_event;
    uint64_t m_pktsSent{0};
    uint64_t m_bytesSent{0};

    uint32_t SamplePoisson(double mean)
    {
        // Knuth algorithm using ns-3 RNG for reproducibility.
        double L = std::exp(-mean);
        uint32_t k = 0;
        double p = 1.0;
        do
        {
            ++k;
            p *= m_uniform->GetValue(0.0, 1.0);
        } while (p > L);
        return k - 1;
    }
};

int
main(int argc, char* argv[])
{
    uint32_t numUes = 1;
    double simTime = 5; // seconds
    double meanInterArrival = 0.005; // seconds
    uint32_t meanPktBytes = 500;
    bool poissonPktSize = true;
    std::string pattern = "F|F|F|F|F|F|F|F|F|F|";

    CommandLine cmd;
    cmd.AddValue("numUes", "Number of UEs", numUes);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("meanInterArrival", "Mean inter-arrival time (s) for Poisson traffic", meanInterArrival);
    cmd.AddValue("meanPktBytes", "Mean packet size (bytes)", meanPktBytes);
    cmd.AddValue("poissonPktSize", "If true, packet size is Poisson-distributed around mean", poissonPktSize);
    cmd.Parse(argc, argv);

    LogComponentEnableAll(LogLevel(LOG_PREFIX_TIME | LOG_PREFIX_FUNC));
    LogComponentEnable("NrPoissonMultiBwpExample", LOG_LEVEL_ALL);
    LogComponentEnable("NrBwpSwitchTriggerHelper", LOG_LEVEL_ALL);
    LogComponentEnable("NrBwpSwitchController", LOG_LEVEL_ALL);
    LogComponentEnable("NrMacSchedulerOfdmaRR", LOG_LEVEL_INFO);
    LogComponentEnable("PacketSink", LOG_LEVEL_INFO);
    LogComponentEnable("UdpL4Protocol", LOG_LEVEL_INFO);
    LogComponentEnable("NrNoBackhaulEpcHelper", LOG_LEVEL_INFO);
    LogComponentEnable("NrEpcUeNas", LOG_LEVEL_INFO);
    LogComponentEnable("NrGnbPhy", LOG_LEVEL_INFO);
    LogComponentEnable("NrUePhy", LOG_LEVEL_INFO);
    LogComponentEnable("NrGnbMac", LOG_LEVEL_INFO);
    LogComponentEnable("NrUeMac", LOG_LEVEL_INFO);
    LogComponentEnable("FlowMonitor", LOG_LEVEL_INFO);
    LogComponentEnable("NrUeRrc", LOG_LEVEL_INFO);
    LogComponentEnable("NrBwpSwitchController", LOG_LEVEL_ALL);

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
    nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaRR::GetTypeId()); // uses external priority hook
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(43.0));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));
    // nrHelper->SetAttribute("UseIdealRrc", BooleanValue(false));
    

    // 3 BWPs: narrow, mid, wide.
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf narrowConf(3.50e9, 5e6, 1);
    CcBwpCreator::SimpleOperationBandConf midConf(3.55e9, 20e6, 1);
    CcBwpCreator::SimpleOperationBandConf wideConf(3.60e9, 80e6, 1);

    OperationBandInfo bandNarrow = ccBwpCreator.CreateOperationBandContiguousCc(narrowConf);
    OperationBandInfo bandMid = ccBwpCreator.CreateOperationBandContiguousCc(midConf);
    OperationBandInfo bandWide = ccBwpCreator.CreateOperationBandContiguousCc(wideConf);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    channelHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    channelHelper->AssignChannelsToBands({bandNarrow, bandMid, bandWide});

    auto allBwps = CcBwpCreator::GetAllBwps({bandNarrow, bandMid, bandWide});
    // nrHelper->SetChannelHelper(channelHelper);
    Ptr<NrBwpSwitchController> switchCtrl = CreateObject<NrBwpSwitchController>();

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHost;
    remoteHost.Create(1);

    InternetStackHelper internet;
    internet.Install(remoteHost);
    internet.Install(ueNodes);

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    Ptr<BwpManagerGnb> gnbBwpMgr =
        DynamicCast<BwpManagerGnb>(gnbDevs.Get(0)->GetObject<NrGnbNetDevice>()->GetComponentCarrierManager());
    switchCtrl->SetGnbManager(gnbBwpMgr);
    // switchCtrl->SetAttribute("EnableInternalPolicy", BooleanValue(true));

    // Connect BSR trace to controller (BWP-resolved).
    gnbBwpMgr->TraceConnectWithoutContext("BsrReport",
                                          MakeCallback(&NrBwpSwitchController::HandleBsr, switchCtrl));

    for(auto i = gnbDevs.Begin(); i != gnbDevs.End(); i++)
    {   
        // setup TDD pattern for each BWP
        NrHelper::GetGnbPhy((*i), 0)->SetAttribute("Pattern", StringValue(pattern));
        NrHelper::GetGnbPhy((*i), 1)->SetAttribute("Pattern", StringValue(pattern));
        NrHelper::GetGnbPhy((*i), 2)->SetAttribute("Pattern", StringValue(pattern));
    }

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2ph.SetChannelAttribute("Delay", StringValue("1ms"));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost.Get(0));

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ipv4Address pgwP2pAddr = internetIpIfaces.GetAddress(0);
    Ipv4Address rhP2pAddr = internetIpIfaces.GetAddress(1);
    NS_LOG_INFO("PGW p2p addr=" << pgwP2pAddr << " remoteHost addr=" << rhP2pAddr);

    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(ueDevs);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost.Get(0)->GetObject<Ipv4>());
    // Route to UE subnet via PGW gateway on p2p link
    Ipv4Address pgwAddr = internetIpIfaces.GetAddress(0); // PGW side of p2p
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                               Ipv4Mask("255.0.0.0"),
                                               pgwAddr,
                                               1);

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Node> ue = ueNodes.Get(i);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ue->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
        NS_LOG_INFO("UE" << i << " IP=" << ueIpIfaces.GetAddress(i));

        Ptr<NrUeNetDevice> ueDev = ueDevs.Get(i)->GetObject<NrUeNetDevice>();
        Ptr<BwpManagerUe> ueMgr = ueDev->GetBwpManager();
        RegisterUeManager(switchCtrl, ueDev);
    }

    uint16_t dlPort = 10000;
    std::vector<Ptr<Application>> sinks;
    std::vector<Ptr<PoissonDlApp>> sources;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ipv4Address ueAddr = ueIpIfaces.GetAddress(i);
        uint16_t port = dlPort + i;
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
        auto sinkApp = sinkHelper.Install(ueNodes.Get(i));
        sinkApp.Start(Seconds(1.0));
        sinkApp.Stop(Seconds(simTime));

        Ptr<Socket> srcSocket =
            Socket::CreateSocket(remoteHost.Get(0), UdpSocketFactory::GetTypeId());

        Ptr<PoissonDlApp> app = CreateObject<PoissonDlApp>();
        app->SetSocket(srcSocket);
        app->SetPeer(InetSocketAddress(ueAddr, port));
        app->SetMeanInterArrival(meanInterArrival);
        app->SetMeanPacketSize(meanPktBytes, poissonPktSize);
        remoteHost.Get(0)->AddApplication(app);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(simTime));

        sinks.push_back(sinkApp.Get(0));
        sources.push_back(app);
    }

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // FlowMonitor stats
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    auto stats = monitor->GetFlowStats();
    NS_LOG_UNCOND("==== FlowMonitor ====");
    for (const auto& kv : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
        NS_LOG_UNCOND("Flow " << kv.first << " " << t.sourceAddress << ":" << t.sourcePort << " -> "
                              << t.destinationAddress << ":" << t.destinationPort << " txBytes="
                              << kv.second.txBytes << " rxBytes=" << kv.second.rxBytes
                              << " throughputMbps=" << (kv.second.rxBytes * 8.0 / (simTime * 1e6)));
    }

    // Summaries
    NS_LOG_UNCOND("==== Simulation Summary ====");
    for (uint32_t i = 0; i < sinks.size(); ++i)
    {
        Ptr<PacketSink> sink = sinks[i]->GetObject<PacketSink>();
        uint64_t rxBytes = sink->GetTotalRx();
        double throughputMbps = (rxBytes * 8.0) / (simTime * 1e6);
        uint64_t txPkts = sources[i]->GetPacketsSent();
        uint64_t txBytes = sources[i]->GetBytesSent();
        NS_LOG_UNCOND("UE" << i << " txPkts=" << txPkts << " txBytes=" << txBytes << " rxBytes=" << rxBytes
                           << " thr=" << throughputMbps << " Mbps");
    }

    Simulator::Destroy();
    return 0;
}
