#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/point-to-point-helper.h"


using namespace ns3;

// === Trace Callbacks ===
void DlCqiReportCallback(uint16_t rnti, uint8_t cqi)
{
    std::cout << Simulator::Now().GetSeconds()
              << "s [CQI] RNTI=" << rnti
              << " CQI=" << (uint32_t)cqi << std::endl;
}

void SinrReportCallback(uint16_t cellId, uint16_t rnti, double rsrp, double sinr, uint8_t componentCarrierId)
{
    std::cout << Simulator::Now().GetSeconds()
              << "s [SINR] RNTI=" << rnti
              << " SINR=" << sinr << " dB"
              << " RSRP=" << rsrp << " dBm"
              << " CellId=" << cellId << std::endl;
}

void PhyRxCallback(ns3::RxPacketTraceParams params)
{
    std::cout << Simulator::Now().GetSeconds()
              << "s [RX] RNTI=" << params.m_rnti
              << " TB_Size=" << params.m_tbSize
              << " MCS=" << (uint32_t)params.m_mcs
              << " SINR=" << params.m_sinr << " dB" << std::endl;
}

int main(int argc, char *argv[])
{
    CommandLine cmd;
    cmd.Parse(argc, argv);

    // Enable logs
    LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
    LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
    //LogComponentEnable("NrUePhy", LOG_LEVEL_INFO);
    //LogComponentEnable("NrGnbPhy", LOG_LEVEL_INFO);
    LogComponentEnable("NrAmc", LOG_LEVEL_INFO);
    //LogComponentEnable("NrGnbMac", LOG_LEVEL_INFO);
    //LogComponentEnable("NrUeMac", LOG_LEVEL_INFO);

    NodeContainer gNbNodes;
    NodeContainer ueNodes;
    NodeContainer remoteHostContainer;
    gNbNodes.Create(1);
    ueNodes.Create(1);
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0, 0, 10));
    positionAlloc->Add(Vector(30, 0, 1.5));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(gNbNodes);
    mobility.Install(ueNodes);
    mobility.Install(remoteHostContainer);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    idealBeamformingHelper->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));

    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 100e6, 1, BandwidthPartInfo::UMa);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    nrHelper->InitializeOperationBand(&band);
    allBwps = CcBwpCreator::GetAllBwps({band});

    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));

    NetDeviceContainer gNbDevs = nrHelper->InstallGnbDevice(gNbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    for (uint32_t i = 0; i < gNbDevs.GetN(); ++i) {
        DynamicCast<NrGnbNetDevice>(gNbDevs.Get(i))->UpdateConfig();
    }

    for (uint32_t i = 0; i < ueDevs.GetN(); ++i) {
        DynamicCast<NrUeNetDevice>(ueDevs.Get(i))->UpdateConfig();
    }

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    InternetStackHelper internet;
    internet.Install(pgw);
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    internet.Install(ueNodes);

    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(ueDevs);
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j) {
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    nrHelper->AttachToClosestGnb(ueDevs, gNbDevs);

    // === Connect PHY Traces ===
    for (uint32_t i = 0; i < ueDevs.GetN(); ++i) {
        Ptr<NrUeNetDevice> ueDev = ueDevs.Get(i)->GetObject<NrUeNetDevice>();
        for (uint32_t j = 0; j < ueDev->GetCcMapSize(); ++j) {
            Ptr<NrUePhy> uePhy = ueDev->GetPhy(j);
            uePhy->TraceConnectWithoutContext("DlCqiReport", MakeCallback(&DlCqiReportCallback));
            uePhy->TraceConnectWithoutContext("ReportCurrentCellRsrpSinr", MakeCallback(&SinrReportCallback));
            uePhy->TraceConnectWithoutContext("RxPacketTrace", MakeCallback(&PhyRxCallback));
        }
    }

    nrHelper->EnableTraces();

    uint16_t dlPort = 1234;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;

    UdpServerHelper dlPacketSinkHelper(dlPort);
    serverApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(0)));

    UdpClientHelper dlClient(ueIpIface.GetAddress(0), dlPort);
    dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(1)));    // Faster traffic
    dlClient.SetAttribute("PacketSize", UintegerValue(1400));         // Larger packets
    dlClient.SetAttribute("MaxPackets", UintegerValue(1000));
    clientApps.Add(dlClient.Install(remoteHost));

    serverApps.Start(Seconds(1.0));
    clientApps.Start(Seconds(1.01));
    serverApps.Stop(Seconds(15.0));
    clientApps.Stop(Seconds(15.0));

    // Print routing table

    Simulator::Stop(Seconds(15.0));

    // Remove Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // Print the routing tables
    for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i)
    {
        Ptr<Node> node = NodeList::GetNode(i);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        if (ipv4)
        {
            std::cout << "==== Routing table for node " << node->GetId() << " ====" << std::endl;
            Ptr<Ipv4RoutingProtocol> routingProtocol = ipv4->GetRoutingProtocol();
            Ptr<Ipv4GlobalRouting> globalRouting = DynamicCast<Ipv4GlobalRouting>(routingProtocol);

            if (globalRouting)
            {
                uint32_t nRoutes = globalRouting->GetNRoutes();
                for (uint32_t j = 0; j < nRoutes; ++j)
                {
                    Ipv4RoutingTableEntry route = globalRouting->GetRoute(j);
                    std::cout << route << std::endl;
                }
            }
            else
            {
                std::cout << "No Ipv4GlobalRouting instance found on node " << node->GetId() << std::endl;
            }
        }
    }

    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
