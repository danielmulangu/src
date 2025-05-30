//Daniel M. Kaseya
//dkaseya@unomaha.edu

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("SimpleWifiWithFlowMonitor");

int main (int argc, char *argv[])
{
  // Enable logging
  LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);

  // Create two nodes
  NodeContainer nodes;
  nodes.Create (2);

  // Set mobility (which is required by Wifi)
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);

  // Set up Wi-Fi
  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211n); //set up the standard
  wifi.SetRemoteStationManager ("ns3::IdealWifiManager"); // choose the right station managger that supports HT (High Throughput)

  YansWifiPhyHelper phy;

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  phy.SetChannel (channel.Create ());//Create Phy

  WifiMacHelper mac;
  Ssid ssid = Ssid ("ns3-simple-wifi");

  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid), "ActiveProbing", BooleanValue (false));
  NetDeviceContainer devices;
  devices.Add (wifi.Install (phy, mac, nodes.Get (0)));

  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
  devices.Add (wifi.Install (phy, mac, nodes.Get (1)));

  // Internet stack
  InternetStackHelper stack;
  stack.Install (nodes);

  // IP addressing
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);

  // UDP Echo Server on node 1
  UdpEchoServerHelper echoServer (9);
  ApplicationContainer serverApps = echoServer.Install (nodes.Get (1));
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (10.0));

  // UDP Echo Client on node 0
  UdpEchoClientHelper echoClient (interfaces.GetAddress (1), 9);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApps = echoClient.Install (nodes.Get (0));
  clientApps.Start (Seconds (2.0));
  clientApps.Stop (Seconds (10.0));

  // Install FlowMonitor
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  // Run simulation
  Simulator::Stop (Seconds (11.0));
  Simulator::Run ();

  // FlowMonitor analysis
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

  for (auto &flow : stats)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (flow.first);
      std::cout << "Flow " << flow.first << " (" << t.sourceAddress << " → " << t.destinationAddress << ")\n";
      std::cout << "  Tx Packets: " << flow.second.txPackets << "\n";
      std::cout << "  Rx Packets: " << flow.second.rxPackets << "\n";
      std::cout << "  Throughput: "
                << (flow.second.rxBytes * 8.0 / (flow.second.timeLastRxPacket.GetSeconds() - flow.second.timeFirstTxPacket.GetSeconds())) / 1000
                << " kbps\n";
    }

  Simulator::Destroy ();
  return 0;
}
