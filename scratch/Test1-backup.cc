//Daniel M. Kaseya
//dkaseya@unomaha.edu

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("SimpleWifiExample");

int main (int argc, char *argv[])
{
  // Enable logging for debugging

    LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
    LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
  // Create two nodes
  NodeContainer nodes;
  nodes.Create (2);
    // Set mobility so YansWifiChannel works properly and avoid the mobility error
    MobilityHelper mobility;
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (nodes);
  // Set up the WiFi device
  WifiHelper wifi;
  //wifi.SetRemoteStationManager ("ns3::AarfWifiManager");

  wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");
  YansWifiPhyHelper phy;
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  phy.SetChannel (channel.Create ());

  WifiMacHelper mac;
  Ssid ssid = Ssid ("ns3-simple-wifi");
  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid), "ActiveProbing", BooleanValue (false));

  // Install WiFi on the nodes
  NetDeviceContainer devices;
  devices.Add (wifi.Install (phy, mac, nodes.Get (0)));  // Access Point
  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
  devices.Add (wifi.Install (phy, mac, nodes.Get (1)));  // Station

  // Install the Internet stack
  InternetStackHelper internet;
  internet.Install (nodes);

  // Assign IP addresses
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  ipv4.Assign (devices);

  // Set up the UDP application (server)
  UdpEchoServerHelper echoServer (9);
  ApplicationContainer serverApps = echoServer.Install (nodes.Get (1));
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (10.0));

  // Set up the UDP client application
  UdpEchoClientHelper echoClient (Ipv4Address ("10.1.1.2"), 9);  // IP address of the server (node 1)
  echoClient.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApps = echoClient.Install (nodes.Get (0));
  clientApps.Start (Seconds (2.0));
  clientApps.Stop (Seconds (10.0));

  // Run the simulation
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}

