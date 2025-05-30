#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/nr-module.h"

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

    // Set RBG configuration - try both 2 and 4 to see which works
    nrHelper->SetGnbMacAttribute("NumRbPerRbg", UintegerValue(4));

    // Configure scheduler
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(false));
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(false));

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

    // Install devices
    NetDeviceContainer gNbDevs = nrHelper->InstallGnbDevice(gNbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    nrHelper->UpdateDeviceConfigs(gNbDevs);
    nrHelper->UpdateDeviceConfigs(ueDevs);


    // Debug output for RB configuration
    for (uint32_t i = 0; i < gNbDevs.GetN(); ++i) {
        Ptr<NrGnbNetDevice> gNbDev = gNbDevs.Get(i)->GetObject<NrGnbNetDevice>();
        for (uint8_t cc = 0; cc < gNbDev->GetCcMapSize(); ++cc) {
            Ptr<NrGnbPhy> phy = gNbDev->GetPhy(cc);
            Ptr<NrGnbMac> mac = gNbDev->GetMac(cc);
            std::cout << "CC " << (uint32_t)cc << " configuration:" << std::endl;
            std::cout << "  Number of RBs: " << phy->GetRbNum() << std::endl;
            std::cout << "  RBG size: " << mac->GetNumRbPerRbg() << std::endl;
            std::cout << "  RBG count: " << (phy->GetRbNum() / mac->GetNumRbPerRbg()) << std::endl;
        }
    }

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


    // Connect CQI trace
    for (uint32_t i = 0; i < ueDevs.GetN(); ++i) {
        Ptr<NrUeNetDevice> ueDev = ueDevs.Get(i)->GetObject<NrUeNetDevice>();
        Ptr<NrUePhy> phy = ueDev->GetPhy(0);
        phy->TraceConnectWithoutContext("CqiFeedbackTrace", MakeCallback(&CqiReportCallback));
    }

    Simulator::Stop(Seconds(1.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}