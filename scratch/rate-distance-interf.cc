//The wifi-rate-adaptation-distance example in example/wireless
//With my (Daniel) personal notes

/*
 * Copyright (c) 2014 Universidad de la República - Uruguay
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Matías Richart <mrichart@fing.edu.uy>
 */

/**
 * This example program is designed to illustrate the behavior of
 * rate-adaptive WiFi rate controls such as Minstrel.  Power-adaptive
 * rate controls can be illustrated also, but separate examples exist for
 * highlighting the power adaptation.
 *
 * This simulation consist of 2 nodes, one AP and one STA.
 * The AP generates UDP traffic with a CBR of 400 Mbps to the STA.
 * The AP can use any power and rate control mechanism and the STA uses
 * only Minstrel rate control.
 * The STA can be configured to move away from (or towards to) the AP.
 * By default, the AP is at coordinate (0,0,0) and the STA starts at
 * coordinate (5,0,0) (meters) and moves away on the x axis by 1 meter every
 * second.
 *
 * The output consists of:
 * - A plot of average throughput vs. distance.
 * - (if logging is enabled) the changes of rate to standard output.
 *
 * Example usage:
 * ./ns3 run "wifi-rate-adaptation-distance --standard=802.11a --staManager=ns3::MinstrelWifiManager
 * --apManager=ns3::MinstrelWifiManager --outputFileName=minstrel"
 *
 * Another example (moving towards the AP):
 * ./ns3 run "wifi-rate-adaptation-distance --standard=802.11a --staManager=ns3::MinstrelWifiManager
 * --apManager=ns3::MinstrelWifiManager --outputFileName=minstrel --stepsSize=1 --STA1_x=-200"
 *
 * Example for HT rates with SGI and channel width of 40MHz:
 * ./ns3 run "wifi-rate-adaptation-distance --staManager=ns3::MinstrelHtWifiManager
 * --apManager=ns3::MinstrelHtWifiManager --outputFileName=minstrelHt --shortGuardInterval=true
 * --channelWidth=40"
 *
 * To enable the log of rate changes:
 * export NS_LOG=RateAdaptationDistance=level_info
 */

//Side note: The program is using Minstrel as the main rate adaptation algorithm for both the
// the AP and the STA (the simulation has 1 of each)

//Side Note: Minstrel is based on ACK feedback and estimates the future success.
// as given rates are based on past SUCCESS


#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/gnuplot.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("RateAdaptationDistance");

/** Node statistics */
class NodeStatistics
{
  public:
    /**
     * Constructor
     * \param aps AP devices
     * \param stas STA devices
     */
    NodeStatistics(NetDeviceContainer aps, NetDeviceContainer stas);

    /**
     * RX callback
     * \param path path
     * \param packet received packet
     * \param from sender
     */
    void RxCallback(std::string path, Ptr<const Packet> packet, const Address& from);
    /**
     * Set node position
     * \param node the node
     * \param position the position
     */
    void SetPosition(Ptr<Node> node, Vector position);
    /**
     * Advance node position
     * \param node the node
     * \param stepsSize the size of a step
     * \param stepsTime the time interval between steps
     */
    void AdvancePosition(Ptr<Node> node, int stepsSize, int stepsTime);
    /**
     * Get node position
     * \param node the node
     * \return the position
     */
    Vector GetPosition(Ptr<Node> node);
    /**
     * \return the gnuplot 2d dataset
     */
    Gnuplot2dDataset GetDatafile();

  private:
    uint32_t m_bytesTotal;     //!< total bytes
    Gnuplot2dDataset m_output; //!< gnuplot 2d dataset
};

NodeStatistics::NodeStatistics(NetDeviceContainer aps, NetDeviceContainer stas)
{
    m_bytesTotal = 0; // Side Note: set the total number of bytes to 0 when the object is created!?
}

void
NodeStatistics::RxCallback(std::string path, Ptr<const Packet> packet, const Address& from)
{
    m_bytesTotal += packet->GetSize(); // Side Note: Update it after a successful packet delivery
}

void
NodeStatistics::SetPosition(Ptr<Node> node, Vector position)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>(); // Side Note: Obtain the Node mobility
    //Important because if it is not done, when assigning the position it will return a null pointer
    // The position is inside the mobility (it is an attribute of the mobility)

    mobility->SetPosition(position); // And set the node position in terms of (X,Y,Z)
}

Vector
NodeStatistics::GetPosition(Ptr<Node> node) //To obtain the node's position
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    return mobility->GetPosition();
}

void
NodeStatistics::AdvancePosition(Ptr<Node> node, int stepsSize, int stepsTime)
{
    Vector pos = GetPosition(node); //Side Note: First Obtain the node's position
    double mbs = ((m_bytesTotal * 8.0) / (1000000 * stepsTime)); //Compute the throughput of the last interval and convert it to
    // Mbit/s
    m_bytesTotal = 0; //Initialize the total num of bytes for next interval
    m_output.Add(pos.x, mbs); //Logging the data throughput (load the data to the file using gnuplot: THe position, Mbits/s)
    pos.x += stepsSize; // Increase the position by the stepsize (in the X direction)
    SetPosition(node, pos); //Set the new position on the node
    Simulator::Schedule(Seconds(stepsTime),
                        &NodeStatistics::AdvancePosition,
                        this,
                        node,
                        stepsSize,
                        stepsTime); //Repeats the same process over again
}

Gnuplot2dDataset
NodeStatistics::GetDatafile() //returns the collected data that was stored during the simulation run.
{
    return m_output;
}

/**
 * Callback for 'Rate' trace source
 *
 * \param oldRate old MCS rate (bits/sec)
 * \param newRate new MCS rate (bits/sec)
 */
void
RateCallback(uint64_t oldRate, uint64_t newRate)
{
    NS_LOG_INFO("Rate " << newRate / 1000000.0 << " Mbps");
}

int
main(int argc, char* argv[])
{
    uint32_t rtsThreshold{65535};
    std::string staManager{"ns3::MinstrelWifiManager"}; //Side Note: The standard based on the minstrel algorithm for High Throughput
    std::string apManager{"ns3::MinstrelWifiManager"};
    std::string standard{"802.11a"}; //Thee wifi standard
    std::string outputFileName{"Minstrel-test9-interference"};
    uint32_t BeMaxAmpduSize{65535};
    bool shortGuardInterval{false};
    uint32_t chWidth{20};
    int ap1_x{0};
    int ap1_y{0};
    int sta1_x{5};
    int sta1_y{0};
    int steps{100};
    int stepsSize{1};
    int stepsTime{1};

    CommandLine cmd(__FILE__);
    cmd.AddValue("staManager", "Rate adaptation manager of the STA", staManager);
    cmd.AddValue("apManager", "Rate adaptation manager of the AP", apManager);
    cmd.AddValue("standard", "Wifi standard (a/b/g/n/ac only)", standard);
    cmd.AddValue("shortGuardInterval",
                 "Enable Short Guard Interval in all stations",
                 shortGuardInterval);
    cmd.AddValue("channelWidth", "Channel width of all the stations", chWidth);
    cmd.AddValue("rtsThreshold", "RTS threshold", rtsThreshold);
    cmd.AddValue("BeMaxAmpduSize", "BE Mac A-MPDU size", BeMaxAmpduSize);
    cmd.AddValue("outputFileName", "Output filename", outputFileName);
    cmd.AddValue("steps", "How many different distances to try", steps);
    cmd.AddValue("stepsTime", "Time on each step", stepsTime);
    cmd.AddValue("stepsSize", "Distance between steps", stepsSize);
    cmd.AddValue("AP1_x", "Position of AP1 in x coordinate", ap1_x);
    cmd.AddValue("AP1_y", "Position of AP1 in y coordinate", ap1_y);
    cmd.AddValue("STA1_x", "Position of STA1 in x coordinate", sta1_x);
    cmd.AddValue("STA1_y", "Position of STA1 in y coordinate", sta1_y);
    cmd.Parse(argc, argv);

    int simuTime = steps * stepsTime;

    if (standard != "802.11a" && standard != "802.11b" && standard != "802.11g" &&
        standard != "802.11n-2.4GHz" && standard != "802.11n-5GHz" && standard != "802.11ac")
    {
        NS_FATAL_ERROR("Standard " << standard << " is not supported by this program");
    }//Side Note, generate an error if the wrong standard is provided in the commandLine

    // Define the APs
    NodeContainer wifiApNodes;
    wifiApNodes.Create(1);

    // Define the STAs
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(1);


    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    // Channel configuration via ChannelSettings attribute can be performed here
    std::string frequencyBand;
    if (standard == "802.11b" || standard == "802.11g" || standard == "802.11n-2.4GHz")
    {
        frequencyBand = "BAND_2_4GHZ";
    }
    else
    {
        frequencyBand = "BAND_5GHZ";
    }
    wifiPhy.Set("ChannelSettings",
                StringValue("{0, " + std::to_string(chWidth) + ", " + frequencyBand + ", 0}"));

    // By default, the CCA sensitivity is -82 dBm, which means if the RSS is
    // below this value, the receiver will reject the Wi-Fi frame.
    // However, we want to allow the rate adaptation to work down to low
    // SNR values.  To allow this, we need to do three things:  1) disable
    // the noise figure (set it to 0 dB) so that the noise level in 20 MHz
    // is around -101 dBm, 2) lower the CCA sensitivity to a value that
    // disables it (e.g. -110 dBm), and 3) disable the Wi-Fi preamble
    // detection model.
    wifiPhy.Set("CcaSensitivity", DoubleValue(-110)); // Keep the threshold at a lower value so that the channel will not be considered..
    // .. busy unless the signal is much stronger.
    wifiPhy.Set("RxNoiseFigure", DoubleValue(0.0)); //Reduce the receiver noise (It will consider a perfectly quiet radio)..
    //.. to perfectly detect weaker signals
    wifiPhy.DisablePreambleDetectionModel(); //“Skip the preamble test,” so even ultra-weak frames get a chance to be processed..
    //.. and counted by the rate-adaptation logic.

    NetDeviceContainer wifiApDevices;
    NetDeviceContainer wifiStaDevices;
    NetDeviceContainer wifiDevices;

    WifiHelper wifi;
    if (standard == "802.11a" || standard == "802.11b" || standard == "802.11g")
    {
        if (standard == "802.11a")
        {
            wifi.SetStandard(WIFI_STANDARD_80211a);
        }
        else if (standard == "802.11b")
        {
            wifi.SetStandard(WIFI_STANDARD_80211b);
        }
        else if (standard == "802.11g")
        {
            wifi.SetStandard(WIFI_STANDARD_80211g);
        }
        WifiMacHelper wifiMac;

        // Configure the STA node
        wifi.SetRemoteStationManager(staManager, "RtsCtsThreshold", UintegerValue(rtsThreshold));

        Ssid ssid = Ssid("AP");
        wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
        wifiStaDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiStaNodes.Get(0)));

        // Configure the AP node
        wifi.SetRemoteStationManager(apManager, "RtsCtsThreshold", UintegerValue(rtsThreshold));

        ssid = Ssid("AP");
        wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        wifiApDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiApNodes.Get(0)));
    }
    else if (standard == "802.11n-2.4GHz" || standard == "802.11n-5GHz" || standard == "802.11ac")
    {
        if (standard == "802.11n-2.4GHz" || standard == "802.11n-5GHz")
        {
            wifi.SetStandard(WIFI_STANDARD_80211n);
        }
        else if (standard == "802.11ac")
        {
            wifi.SetStandard(WIFI_STANDARD_80211ac);
        }

        WifiMacHelper wifiMac;

        // Configure the STA node
        wifi.SetRemoteStationManager(staManager, "RtsCtsThreshold", UintegerValue(rtsThreshold));

        Ssid ssid = Ssid("AP");
        /*About SSID
         * When a STA receives a data frame, management frame, or beacon,
         * its MAC compares the frame’s SSID against the one stored in its StaWifiMac;
         * mismatching frames are silently ignored.
         */
        wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
        wifiStaDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiStaNodes.Get(0)));

        // Configure the AP node
        wifi.SetRemoteStationManager(apManager, "RtsCtsThreshold", UintegerValue(rtsThreshold));

        ssid = Ssid("AP");
        wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        wifiApDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiApNodes.Get(0)));

        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize",
                    UintegerValue(BeMaxAmpduSize));
    }

    wifiDevices.Add(wifiStaDevices);
    wifiDevices.Add(wifiApDevices);

    // Set guard interval
    Config::Set(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported",
        BooleanValue(shortGuardInterval));

    //Create a interference node
    NodeContainer interfererNode;
    interfererNode.Create(1);

    // Install device
    NetDeviceContainer interfererDevice;
    Ssid ssid = Ssid("INTERFERER");
    WifiMacHelper interfererMac;
    interfererMac.SetType("ns3::AdhocWifiMac");
    interfererDevice = wifi.Install(wifiPhy, interfererMac, interfererNode.Get(0));

    // Set mobility - Place it near the STA or midway between STA and AP
    MobilityHelper interfererMobility;
    Ptr<ListPositionAllocator> interfererAlloc = CreateObject<ListPositionAllocator>();
    interfererAlloc->Add(Vector(3.0, 0.0, 0.0)); // Adjust as needed
    interfererMobility.SetPositionAllocator(interfererAlloc);
    interfererMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    interfererMobility.Install(interfererNode.Get(0));

    // Configure the mobility.
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    // Initial position of AP and STA
    positionAlloc->Add(Vector(ap1_x, ap1_y, 0.0));
    positionAlloc->Add(Vector(sta1_x, sta1_y, 0.0));
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(wifiApNodes.Get(0));
    mobility.Install(wifiStaNodes.Get(0));

    // Statistics counter
    NodeStatistics atpCounter = NodeStatistics(wifiApDevices, wifiStaDevices);

    // Move the STA by stepsSize meters every stepsTime seconds
    Simulator::Schedule(Seconds(0.5 + stepsTime),
                        &NodeStatistics::AdvancePosition,
                        &atpCounter,
                        wifiStaNodes.Get(0),
                        stepsSize,
                        stepsTime);


    // Configure the IP stack AP + STA
    InternetStackHelper stack;
    stack.Install(wifiApNodes);
    stack.Install(wifiStaNodes);
    stack.Install(interfererNode);
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i = address.Assign(wifiDevices);
    Ipv4Address sinkAddress = i.GetAddress(0);
    uint16_t port = 9;
    //Interference
    Ipv4AddressHelper address1;
    address1.SetBase("10.1.2.0", "255.255.255.0");
    address.Assign(interfererDevice);

    // Configure the CBR generator
    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress, port));
    ApplicationContainer apps_sink = sink.Install(wifiStaNodes.Get(0));

    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress, port));
    onoff.SetConstantRate(DataRate("400Mbps"), 1420);
    onoff.SetAttribute("StartTime", TimeValue(Seconds(0.5)));
    onoff.SetAttribute("StopTime", TimeValue(Seconds(simuTime)));
    ApplicationContainer apps_source = onoff.Install(wifiApNodes.Get(0));

    apps_sink.Start(Seconds(0.5));
    apps_sink.Stop(Seconds(simuTime));

    // Interferer Setup
    OnOffHelper interfererApp("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address("255.255.255.255"),port));
    // interfererApp.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]")); //Might be changed later on, just to check if it works or not
    interfererApp.SetAttribute("StartTime", TimeValue(Seconds(0.5)));
    interfererApp.SetAttribute("StopTime", TimeValue(Seconds(simuTime)));
    interfererApp.SetAttribute("DataRate", StringValue("400Mbps"));
    interfererApp.SetAttribute("PacketSize", UintegerValue(1420));

    ApplicationContainer apps_interferer = interfererApp.Install(interfererNode.Get(0));
    apps_interferer.Start(Seconds(0.5));
    apps_interferer.Stop(Seconds(simuTime));

    //------------------------------------------------------------
    //-- Setup stats and data collection
    //--------------------------------------------

    // Register packet receptions to calculate throughput
    Config::Connect("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",
                    MakeCallback(&NodeStatistics::RxCallback, &atpCounter));

    // Callbacks to print every change of rate
    Config::ConnectWithoutContextFailSafe(
        "/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + apManager + "/Rate",
        MakeCallback(RateCallback));

    Simulator::Stop(Seconds(simuTime));
    Simulator::Run();

    std::ofstream outfile("throughput-" + outputFileName + ".plt");
    Gnuplot gnuplot = Gnuplot("throughput-" + outputFileName + ".eps", "Throughput");
    gnuplot.SetTerminal("post eps color enhanced");
    gnuplot.SetLegend("Distance (meters)", "Throughput (Mb/s)");
    gnuplot.SetTitle("Throughput (AP to STA) vs distance");
    gnuplot.AddDataset(atpCounter.GetDatafile());
    gnuplot.GenerateOutput(outfile);

    Simulator::Destroy();

    return 0;
}
