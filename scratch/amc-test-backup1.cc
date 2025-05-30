#include "../src/lte/helper/point-to-point-epc-helper.h"

#include "ns3/applications-module.h" //For PacketSinkHelper and UdpClient
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-helper.h"  // For NrHelper and GetEpcHelper()
#include "ns3/nr-point-to-point-epc-helper.h"
using namespace ns3;

Ptr<NrAmc> g_amcModel;

//Callback Function
void CqiReportCallback(uint16_t rnti, uint8_t cqi, uint8_t mcs, uint8_t ri)
{
    uint8_t recomputedMcs = g_amcModel->GetMcsFromCqi(cqi);
    std::cout << Simulator::Now().GetSeconds() << "s CQI=" << (uint32_t)cqi
              << " → MCS=" << (uint32_t)recomputedMcs << std::endl;
}

int main(int argc, char *argv[])
{
    CommandLine cmd;
    cmd.Parse(argc, argv);
    LogComponentEnable("NrUePhy", LOG_LEVEL_INFO);
    LogComponentEnable("NrAmc", LOG_LEVEL_INFO);
    //LogComponentEnable("NrSpectrumPhy", LOG_LEVEL_INFO);  // Optional but helpful
    //LogComponentEnable("NrGnbPhy", LOG_LEVEL_INFO);       // Optional but helpful

    // Create nodes
    NodeContainer gNbNodes;
    NodeContainer ueNodes;
    gNbNodes.Create(1);
    ueNodes.Create(1);

    // Mobility
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gNbNodes);
    mobility.Install(ueNodes);

    // NR helper configuration

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetGnbMacAttribute ("NumRbPerRbg", UintegerValue (2));
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    //Possible Fix
    g_amcModel = CreateObject<NrAmc>();

    // Configure scheduler
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(false));
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(false));

    nrHelper->SetGnbMacAttribute("NumRbPerRbg", UintegerValue(1));

    // Initialize channel and pathloss
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    // Configure AMC model
    g_amcModel = CreateObject<NrAmc>();
    g_amcModel->SetAmcModel(NrAmc::ErrorModel);
    g_amcModel->SetDlMode();
    g_amcModel->SetNumRefScPerRb(1);

    // Bandwidth part configuration
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;

    // 20MHz bandwidth, numerology 1, UMa scenario
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1, BandwidthPartInfo::UMa);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);



    // Initialize operation band
    nrHelper->InitializeOperationBand(&band);
    allBwps = CcBwpCreator::GetAllBwps({band});
    //nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(0));

    // Install devices
    NetDeviceContainer gNbDevs = nrHelper->InstallGnbDevice(gNbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->UpdateDeviceConfigs(gNbDevs);
    nrHelper->UpdateDeviceConfigs(ueDevs);
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(2.0));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(2.0));

    std::cout << "Successfully installed the devices";
    //Debugging Purposes

    // IP stack
    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(ueDevs);
    std::cout << "\nSuccessfully installed the IP interfaces ";

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
    dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(100)));
    dlClient.SetAttribute("MaxPackets", UintegerValue(1000));
    ApplicationContainer dlClientApp = dlClient.Install(gNbNodes.Get(0));
    dlClientApp.Start(Seconds(1.1));


    // Connect CQI trace
    // for (uint32_t i = 0; i < ueDevs.GetN(); ++i) {
    //     Ptr<NrUeNetDevice> ueDev = ueDevs.Get(i)->GetObject<NrUeNetDevice>();
    //     Ptr<NrUePhy> phy = ueDev->GetPhy(0);
    //     phy->TraceConnectWithoutContext("CqiFeedbackTrace", MakeCallback(&CqiReportCallback));
    // }
    Ptr<NrUeNetDevice> ueDev = ueDevs.Get(0)->GetObject<NrUeNetDevice>();
    std::cout << "\nUE RRC State: " << ueDev->GetRrc()->GetState() << std::endl;
    Simulator::Stop(Seconds(10.0));
    Simulator::Run();
    std::cout << Simulator::Now().GetSeconds() << " seconds" << std::endl;
    Simulator::Destroy();

    return 0;
}
