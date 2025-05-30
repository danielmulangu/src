#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
using namespace ns3;

Ptr<NrAmc> g_amcModel;

// Callback for DL CQI reports
void DlCqiReportCallback(uint16_t rnti, uint8_t cqi)
{
    if (g_amcModel) {
        uint8_t mcs = g_amcModel->GetMcsFromCqi(cqi);
        std::cout << Simulator::Now().GetSeconds() << "s RNTI=" << rnti
                  << " DL_CQI=" << (uint32_t)cqi
                  << " MCS=" << (uint32_t)mcs << std::endl;
    }
}

// Callback for SINR reports - simplified
void SinrReportCallback(uint16_t cellId, uint16_t rnti, double rsrp, double sinr, uint8_t componentCarrierId)
{
    std::cout << Simulator::Now().GetSeconds() << "s RNTI=" << rnti
              << " RSRP=" << rsrp << " dBm"
              << " SINR=" << sinr << " dB"
              << " CellId=" << cellId << std::endl;
}

// Callback for TX data trace
void TxDataCallback(Time time)
{
    std::cout << Simulator::Now().GetSeconds() << "s TX_Data at time=" << time.GetSeconds() << "s" << std::endl;
}

// Callback for detailed PHY stats - using correct trace parameters
void PhyRxCallback(RxPacketTraceParams params)
{
    std::cout << Simulator::Now().GetSeconds() << "s PHY_RX: TB_Size=" << params.m_tbSize
              << " MCS=" << (uint32_t)params.m_mcs
              << " SINR=" << params.m_sinr << " dB"
              << " RNTI=" << params.m_rnti << std::endl;
}

int main(int argc, char *argv[])
{
    CommandLine cmd;
    cmd.Parse(argc, argv);

    // Enable logging for relevant components
    LogComponentEnable("NrUePhy", LOG_LEVEL_INFO);
    LogComponentEnable("NrGnbPhy", LOG_LEVEL_INFO);
    LogComponentEnable("NrUeMac", LOG_LEVEL_INFO);
    LogComponentEnable("NrGnbMac", LOG_LEVEL_INFO);

    // Create nodes
    NodeContainer gNbNodes;
    NodeContainer ueNodes;
    gNbNodes.Create(1);
    ueNodes.Create(1);

    // Mobility - place UE 100m from gNB
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0, 0, 0));    // gNB position
    positionAlloc->Add(Vector(100, 0, 0));  // UE position
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(gNbNodes);
    mobility.Install(ueNodes);

    // NR helper configuration
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    // Configure AMC model
    g_amcModel = CreateObject<NrAmc>();
    g_amcModel->SetAmcModel(NrAmc::ShannonModel);
    g_amcModel->SetDlMode();
    g_amcModel->SetNumRefScPerRb(1);

    // Configure scheduler
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(false));
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(false));
    nrHelper->SetGnbMacAttribute("NumRbPerRbg", UintegerValue(4));

    // Initialize channel and pathloss
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    // Bandwidth part configuration - 20MHz with numerology 1
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1, BandwidthPartInfo::UMa);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // Initialize operation band
    nrHelper->InitializeOperationBand(&band);
    allBwps = CcBwpCreator::GetAllBwps({band});

    // Install devices
    NetDeviceContainer gNbDevs = nrHelper->InstallGnbDevice(gNbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Set power levels
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(30.0)); // 30 dBm for gNB
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(25.0));  // 25 dBm for UE
    nrHelper->UpdateDeviceConfigs(gNbDevs);
    nrHelper->UpdateDeviceConfigs(ueDevs);

    // IP stack
    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(ueDevs);

    // Set default gateway
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j) {
        Ptr<Ipv4> ipv4 = ueNodes.Get(j)->GetObject<Ipv4>();
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4->GetRoutingProtocol()->GetObject<Ipv4StaticRouting>();
        if (ueStaticRouting) {
            ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
        }
    }

    // Attach UEs to gNBs
    nrHelper->AttachToClosestGnb(ueDevs, gNbDevs);

    // Add UDP traffic between gNB and UE
    uint16_t dlPort = 1234;
    PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                     InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    ApplicationContainer dlSinkApp = dlPacketSinkHelper.Install(ueNodes.Get(0));
    dlSinkApp.Start(Seconds(1.0));

    UdpClientHelper dlClient(ueIpIfaces.GetAddress(0), dlPort);
    dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(10))); // More frequent traffic
    dlClient.SetAttribute("MaxPackets", UintegerValue(1000));
    dlClient.SetAttribute("PacketSize", UintegerValue(1024)); // Larger packets
    ApplicationContainer dlClientApp = dlClient.Install(gNbNodes.Get(0));
    dlClientApp.Start(Seconds(1.1));

    // Connect trace sources for UE PHY - focus on working traces
    for (uint32_t i = 0; i < ueDevs.GetN(); ++i) {
        Ptr<NrUeNetDevice> ueDev = ueDevs.Get(i)->GetObject<NrUeNetDevice>();
        for (uint32_t j = 0; j < ueDev->GetCcMapSize(); ++j) {
            Ptr<NrUePhy> uePhy = ueDev->GetPhy(j);

            // Connect to DL CQI trace
            uePhy->TraceConnectWithoutContext("DlCqiReport", MakeCallback(&DlCqiReportCallback));

            // Connect to SINR trace
            uePhy->TraceConnectWithoutContext("ReportCurrentCellRsrpSinr", MakeCallback(&SinrReportCallback));
        }
    }

    // Enable NR traces for comprehensive logging
    nrHelper->EnableTraces();

    Simulator::Stop(Seconds(30.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}