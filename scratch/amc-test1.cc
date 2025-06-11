#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/point-to-point-helper.h"
#include <ns3/antenna-module.h>
#include <cmath>
#include <chrono>

using namespace ns3;

int test_counter=0;

//Map SINR to CQI
uint8_t MapSinrToCqi(double sinrDb)
{
    //  Convert SINR from dB to linear scale
    double sinrLinear = std::pow(10.0, sinrDb / 10.0);

    //  Use the helper to get a spectrum model
    // For simplicity, use 1 RB, 3.5 GHz, 15 kHz subcarrier spacing
    uint32_t numRbs = 1;
    double centerFrequency = 3.5e9;    // 3.5 GHz
    double subcarrierSpacing = 15e3;   // 15 kHz
    auto spectrumModel = ns3::NrSpectrumValueHelper::GetSpectrumModel(numRbs, centerFrequency, subcarrierSpacing);

    // Create the SINR SpectrumValue with one RB
    ns3::Ptr<ns3::SpectrumValue> sinrSpec = ns3::Create<ns3::SpectrumValue>(spectrumModel);
    (*sinrSpec)[0] = sinrLinear;

    // Create the AMC object
    auto amc = ns3::CreateObject<ns3::NrAmc>();
    amc->SetAmcModel(ns3::NrAmc::ErrorModel);
    amc->SetDlMode();
    amc->SetNumRefScPerRb(1);

    // Compute CQI
    uint8_t mcs;
    uint8_t cqi = amc->CreateCqiFeedbackWbTdma(*sinrSpec, mcs);
    //std::cout << "\tcqi: " << cqi << std::endl;
    return cqi;
}


//Callback Function
void DlCtrlSinrCallback(uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId)
{
    // Create and configure the AMC model
    Ptr<NrAmc> amc = CreateObject<NrAmc> ();
    // Select error-based AMC algorithm
    amc->SetAmcModel (NrAmc::ErrorModel);
    // Operate in downlink mode
    amc->SetDlMode ();
    // Default DM-RS symbols per RB = 1
    amc->SetNumRefScPerRb (1);
    // Get CQI from SINR
   uint8_t cqi= MapSinrToCqi(sinr);
    // Get MCS from CQI
    uint8_t mcs = amc->GetMcsFromCqi (cqi);
    const uint32_t numRb = 50;  // Resource Blocks allocated

    // Calculate Transport Block size (layers=1)
    uint32_t tbs = amc->CalculateTbSize (mcs, 1, numRb);
    std::cout << "Time " << Simulator::Now().GetSeconds()
              << "s:   UE RNTI=" << rnti
              << ",   CellId=" << cellId
              << ",   DL Ctrl SINR=" << sinr << " dB,"
              << "   CQI " <<static_cast<uint32_t>(cqi)
              << "   MCS " << static_cast<uint32_t>(mcs)
              << "   TB Size " << tbs
              << std::endl;
    test_counter++;
}


//Position
static Ptr<ListPositionAllocator>
GetGnbPositions(double gNbHeight)
{
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0, 0.0, gNbHeight));

    return pos;
}

static Ptr<ListPositionAllocator>
GetUePositions(double ueY, double ueHeight)
{
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0, ueY, ueHeight));

    return pos;
}


static std::vector<uint64_t> packetsTime;

static void
PrintRxPkt([[maybe_unused]] std::string context, Ptr<const Packet> pkt)
{
    // ASSUMING ONE UE
    SeqTsHeader seqTs;
    pkt->PeekHeader(seqTs);
    packetsTime.push_back((Simulator::Now() - seqTs.GetTs()).GetMicroSeconds());
}

//Getting the Nodes positions (for debugging purposes)
static Vector GetFinalPosition (Ptr <Node> simNode)
{
 Ptr<MobilityModel> mobility = simNode->GetObject<MobilityModel> ();
    if (!mobility)
    {
        NS_FATAL_ERROR("Node has no mobility");
    }
    return mobility->GetPosition();
}
//Printing the Nodes positions (For debugging purposes, again)
static void PrintPositions(NodeContainer sim_node)
{
    for (uint32_t i=0; i< sim_node.GetN();++i)
    {
        Vector position_s = GetFinalPosition(sim_node.Get(i));
        std::cout <<"Position (X,Y,Z)" << position_s << std::endl;
    }
}

int
main (int argc, char *argv[])
{
  // Simulation parameters
    double simDuration = 30.0; // seconds
    double totalTxPower = 3;
    uint16_t numerologyBwp = 4;
    double centralFrequencyBand = 28e9;
    double bandwidthBand = 100e6;
    double ueY = 20.0;
    double gNbHeight = 10.0;
    double ueHeight = 20.0;
    uint32_t pktSize = 500;
    const uint8_t gNbNum = 1;
    const uint8_t ueNum = 1;
    Time udpAppStartTime = MilliSeconds(1000);
    Time packetInterval = MilliSeconds(200);
    Time updateChannelInterval = MilliSeconds(0); // no channel updates to test AMC

    std::string errorModel = "ns3::NrEesmCcT1";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time", simDuration);

    cmd.AddValue("errorModelType",
                 "Error model type: ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1, "
                 "ns3::NrEesmIrT2, ns3::NrLteMiErrorModel",
                 errorModel);
    cmd.AddValue("ueY", "Y position of any UE", ueY);
    cmd.AddValue("pktSize", "Packet Size", pktSize);

    cmd.Parse(argc, argv);

    uint32_t packets = (simDuration - udpAppStartTime.GetSeconds()) / packetInterval.GetSeconds();
    NS_ABORT_IF(packets == 0);

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    Config::SetDefault("ns3::NrAmc::ErrorModelType", TypeIdValue(TypeId::LookupByName(errorModel)));
    Config::SetDefault("ns3::NrAmc::AmcModel",
                       EnumValue(NrAmc::ErrorModel));

    // Create one gNB node and one UE node
  NodeContainer gnbNodes; gnbNodes.Create (gNbNum);
  NodeContainer ueNodes;  ueNodes.Create (ueNum);

    //Position: Set the positions of the nodes using the height and coordinates of the two nodes.
    Ptr<ListPositionAllocator> gNbPositionAlloc = GetGnbPositions(gNbHeight);
    Ptr<ListPositionAllocator> uePositionAlloc = GetUePositions(ueY, ueHeight);

  // Mobility
  MobilityHelper mobility;
    MobilityHelper uemob;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator(gNbPositionAlloc);
  mobility.Install (gnbNodes);
    uemob.SetMobilityModel ("ns3::ConstantVelocityMobilityModel"); // Set the model  for ueNode
    uemob.Install (ueNodes);
    std::cout <<"UE initial position:"<<std::endl;
    PrintPositions(ueNodes);
    std:: cout<<"GNb initial position:"<<std::endl;
    PrintPositions(gnbNodes);
    for (uint n = 0; n < ueNodes.GetN (); n++)
    {
        Ptr<ConstantVelocityMobilityModel> mob = ueNodes.Get(n)->GetObject<ConstantVelocityMobilityModel>();
        // Set the desired velocity
        mob->SetVelocity(Vector(1.0, 2.0, 0.0)); // Moving at 1 m/s in the x direction and 5 m/s in the y direction
    }

  // Set up a minimal 5G “core” (EPC/NGC) and the NR helper
  Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();

  nrHelper->SetBeamformingHelper(idealBeamformingHelper);
  nrHelper->SetEpcHelper (nrEpcHelper);


  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1;



  // Create a “single‐component‐carrier, single‐BWP” operation‐band configuration:
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequencyBand,
                                                     bandwidthBand,
                                                     numCcPerBand,
                                                     BandwidthPartInfo::UMi_StreetCanyon);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
 // OperationBandInfo opBand = ccBwpCreator.CreateOperationBandContiguousCc (bandConf);


    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(updateChannelInterval));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

  nrHelper->InitializeOperationBand(&band);
  allBwps = CcBwpCreator::GetAllBwps({band});

  Packet::EnableChecking();
  Packet::EnableChecking();

    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                           TypeIdValue(DirectPathBeamforming::GetTypeId()));

    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Scheduler
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(false));
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(false));

    // Error Model: UE and GNB with same spectrum error model.
    nrHelper->SetUlErrorModel(errorModel);
    nrHelper->SetDlErrorModel(errorModel);

    // Both DL and UL AMC will have the same model behind.
    nrHelper->SetGnbDlAmcAttribute(
        "AmcModel",
        EnumValue(NrAmc::ShannonModel)); // NrAmc::ShannonModel or NrAmc::ErrorModel
    nrHelper->SetGnbUlAmcAttribute(
        "AmcModel",
        EnumValue(NrAmc::ErrorModel)); // NrAmc::ShannonModel or NrAmc::ErrorModel

    uint32_t bwpId = 0;

    // gNb routing between Bearer and bandwidh part
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpId));

    // Ue routing between Bearer and bandwidth part
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpId));


  NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice (gnbNodes, allBwps);
  NetDeviceContainer ueNetDev  = nrHelper->InstallUeDevice  (ueNodes, allBwps);

    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("Numerology", UintegerValue(numerologyBwp));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(totalTxPower));


    for (auto it = gnbNetDev.Begin(); it != gnbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }

    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // create the internet and install the IP stack on the UE
    // get SGW/PGW and create a single RemoteHost
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // connect a remoteHost to pgw. Setup routing too
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    // Set the default gateway for the UEs
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // assign IP address to UEs, and install UDP downlink applications
    uint16_t dlPort = 1234;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;

    ApplicationContainer clientAppsEmbb;
    ApplicationContainer serverAppsEmbb;

    UdpServerHelper dlPacketSinkHelper(dlPort);
    serverApps.Add(dlPacketSinkHelper.Install(ueNodes));

    // configure here UDP traffic
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j)
    {
        UdpClientHelper dlClient(ueIpIface.GetAddress(j), dlPort);
        dlClient.SetAttribute("MaxPackets", UintegerValue(packets));
        dlClient.SetAttribute("PacketSize", UintegerValue(pktSize));
        dlClient.SetAttribute("Interval", TimeValue(packetInterval));

        clientApps.Add(dlClient.Install(remoteHost));
    }

    for (uint32_t j = 0; j < serverApps.GetN(); ++j)
    {
        Ptr<UdpServer> client = DynamicCast<UdpServer>(serverApps.Get(j));
        NS_ASSERT(client != nullptr);
        std::stringstream ss;
        ss << j;
        client->TraceConnect("Rx", ss.str(), MakeCallback(&PrintRxPkt));
    }

    // start UDP server and client apps
    serverApps.Start(udpAppStartTime);
    clientApps.Start(udpAppStartTime);
    auto start = std::chrono::steady_clock::now();
    serverApps.Stop(Seconds(simDuration));
    clientApps.Stop(Seconds(simDuration));

    // attach UEs to the closest gNB
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);

    // enable the traces provided by the nr module
     nrHelper->EnableTraces();
    Ptr<NrUePhy> uePhy = nrHelper->GetUePhy(ueNetDev.Get(0), 0);
    uePhy->TraceConnectWithoutContext("DlCtrlSinr", MakeCallback(&DlCtrlSinrCallback));

  Simulator::Stop (Seconds (simDuration));


  Simulator::Run ();
    std::cout <<"UE initial position:"<<std::endl;
    PrintPositions(ueNodes);
    std:: cout<<"GNb initial position:"<<std::endl;
    PrintPositions(gnbNodes);

    uint64_t sum = 0;
    uint32_t cont = 0;
    for (auto& v : packetsTime)
    {
        if (v < 100000)
        {
            sum += v;
            cont++;
        }
    }
    std::cout << "Packets received: " << packetsTime.size() << std::endl;
    std::cout << "Counter (packets not affected by reordering): " << +cont << std::endl;

    for (auto it = serverApps.Begin(); it != serverApps.End(); ++it)
    {
        uint64_t recv = DynamicCast<UdpServer>(*it)->GetReceived();
        std::cout << "Sent: " << packets
                  << "\nRecv: " << recv
                  << "\nLost: " << packets - recv
                  << " pkts, ( " << (static_cast<double>(packets - recv) / packets) * 100.0
                  << " % )" << std::endl;

    }
    std::cout <<"UE Final position:"<<std::endl;
    PrintPositions(ueNodes);
    std::cout <<"GNb Final position:"<<std::endl;
    PrintPositions(gnbNodes);
  Simulator::Destroy ();
    auto end = std::chrono::steady_clock::now();
    std::cout << "Running time: "
                << std::chrono::duration_cast<std::chrono::seconds>(end - start)
                  << std::endl;
    if (test_counter!=0){std::cout << "\nCounter triggered"<< std::endl;}
    else {std::cout << "Counter not triggered"<< std::endl;}
  return 0;
}
