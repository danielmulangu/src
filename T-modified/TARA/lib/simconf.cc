//Copyright (C) 2023, INESC TEC
//This file is part of TARA https://gitlab.inesctec.pt/pub/ctm-win/tara.
//
//TARA is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.
//
//TARA is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with TARA.  If not, see <http://www.gnu.org/licenses/>.

#include "simconf.h"
#include "tara.h"
#include <ns3/log.h>
#include <ns3/wifi-module.h>
#include <ns3/core-module.h>
#include <ns3/internet-module.h>
#include <ns3/applications-module.h>
#include <ns3/mobility-module.h>
#include "ns3/wifi-mac-header.h"


namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("simconf");

//Debugging purposes
uint64_t txPacketsFAPtoFGW = 0;
uint64_t rxPacketsFGWfromFAP = 0;

uint64_t txPacketsFGWtoBKH = 0;
uint64_t rxPacketsBKHfromFGW = 0;

uint64_t txPacketsInterferer = 0;
uint64_t totaltxPackets =0;
double currentSnrBkh = 0.0;

//Sink
uint32_t g_totalFapPackets = 0;
uint32_t g_totalInterferencePackets = 0;
ns3::Ipv4Address g_fapIp;
ns3::Ipv4Address g_interferenceIp;

void ReceiveInterference(Ptr<const Packet> packet, const Address &from)
{
    InetSocketAddress address = InetSocketAddress::ConvertFrom(from);
    Ipv4Address senderIp = address.GetIpv4();

    if (senderIp == g_fapIp)
    {
        g_totalFapPackets++;
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s: Packet from FAP (" << g_totalFapPackets << ")");
    }
    else if (senderIp == g_interferenceIp)
    {
        g_totalInterferencePackets++;
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s: Packet from Interferer (" << g_totalInterferencePackets << ")");
    }
    else
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s: Packet from UNKNOWN IP: " << senderIp);
    }
}


// /
void CountTotalTx (std:: string context, Ptr<const Packet> packet)
{
    totaltxPackets++;
}
void CountTxFAPtoFGW(std::string context, Ptr<const Packet> packet) {
    txPacketsFAPtoFGW++;
    //NS_LOG_UNCOND("[DEBUG] TX packet counted from FAP to FGW");
}
void CountRxFGWfromFAP(std::string context, Ptr<const Packet> packet) {
    rxPacketsFGWfromFAP++;
   // NS_LOG_UNCOND("[DEBUG] TX packet counted from FGW to FAP");
}

void CountTxFGWtoBKH(std::string context, Ptr<const Packet> packet) {
    txPacketsFGWtoBKH++;
   // NS_LOG_UNCOND("[DEBUG] TX packet counted from FGW to BKH");
}
void CountRxBKHfromFGW(std::string context, Ptr<const Packet> packet) {
    rxPacketsBKHfromFGW++;
   // NS_LOG_UNCOND("[DEBUG] TX packet counted from BKH to FGW");
}

void CountTxInterference(std::string context, Ptr<const Packet> packet) {
    txPacketsInterferer++;
}


void
SnifferRxCallback(Ptr<const Packet> packet,
                  uint16_t channelFreqMhz,
                  WifiTxVector txVector,
                  MpduInfo aMpdu,
                  SignalNoiseDbm snr,
                  uint16_t staId)
{
    currentSnrBkh = snr.signal - snr.noise; // SNR (in dB)
}

void PrintDeviceSummary()
{
    NS_LOG_UNCOND("========== Device Summary ==========");
    for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i)
    {
        Ptr<Node> node = NodeList::GetNode(i);
        NS_LOG_UNCOND("Node [" << i << "] has " << node->GetNDevices() << " devices:");
        for (uint32_t j = 0; j < node->GetNDevices(); ++j)
        {
            Ptr<NetDevice> dev = node->GetDevice(j);
            NS_LOG_UNCOND("  - Device[" << j << "]: " << dev->GetInstanceTypeId().GetName());
        }
    }
    NS_LOG_UNCOND("====================================");
}

// /

  void
  configNodeMobility()
  {
    NodeContainer nodes = NodeContainer::GetGlobal();

    Vector max_box = Vector(1000,1000,0);
    double flight_duration = 0, new_interval=30, config_moment;
    int start_seconds = 5; //WARNING: It starts at 5 seconds sim time
    bool condition;

    MobilityHelper mobility = configInitPosition(max_box);

    mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
    mobility.Install (nodes);

    Ptr<UniformRandomVariable> random_num = CreateObject<UniformRandomVariable> ();
    struct NodeMovInfo fap;
    Vector future_pos, velocity;

    uint16_t times_called = (100-start_seconds)/new_interval; //300 = sim duration

    for(uint8_t i=0 ; i < nodes.GetN() ; i++)
    {

      Ptr<ConstantVelocityMobilityModel> mob_model = DynamicCast<ConstantVelocityMobilityModel> (nodes.Get(i)->GetObject<MobilityModel>());
      Vector curr_pos = mob_model->GetPosition();

      if(i==0 || i==nodes.GetN()-1 || i==2) //assuming Bkh is defined as first node id and FGW as last node id always
        continue;

        if(i == 2) { // Interference node
            // Configure random movement similar to FAP but with different parameters
            do {
                future_pos.x = double(RoundIntToMultiple(random_num->GetInteger(0, max_box.x), 1));
                future_pos.y = double(RoundIntToMultiple(random_num->GetInteger(0, max_box.y), 1));
                future_pos.z = double(RoundIntToMultiple(random_num->GetInteger(0, max_box.z), 1));

                flight_duration = random_num->GetInteger(5, new_interval); // Longer movements
                velocity.x = double((future_pos.x-curr_pos.x)/flight_duration);
                velocity.y = double((future_pos.y-curr_pos.y)/flight_duration);
                velocity.z = double((future_pos.z-curr_pos.z)/flight_duration);

                int abs_velocity = sqrt(pow(abs(velocity.x),2)+pow(abs(velocity.y),2)+pow(abs(velocity.z),2));
                condition = (abs_velocity >= 5 && abs_velocity <= 10); // Variable velocity
            } while(!condition);
        }
      for(uint8_t j=0 ; j < times_called ; j++)
      {
        do{
          future_pos.x = double(RoundIntToMultiple(random_num->GetInteger(0, max_box.x), 1));
          future_pos.y = double(RoundIntToMultiple(random_num->GetInteger(0, max_box.y), 1));
          future_pos.z = double(RoundIntToMultiple(random_num->GetInteger(0, max_box.z), 1));

          flight_duration= random_num->GetInteger(0, new_interval);

          velocity.x = double((future_pos.x-curr_pos.x)/flight_duration);
          velocity.y = double((future_pos.y-curr_pos.y)/flight_duration);
          velocity.z = double((future_pos.z-curr_pos.z)/flight_duration);

          int abs_velocity = sqrt(pow(abs(velocity.x),2)+pow(abs(velocity.y),2)+pow(abs(velocity.z),2));
          condition = (abs_velocity == 8); //fixed velocity @ ~30km/h -> 8m/s

        }while(!condition);

        fap.current_pos = curr_pos;
        fap.velocity = velocity;
        fap.flight_duration = flight_duration;
        fap.future_pos = future_pos;

        curr_pos = future_pos; //updates curr_pos to future position
        config_moment = (j * new_interval)+start_seconds;

        //FAP Positioning
        Simulator::Schedule(Seconds(config_moment), &SetNodeMovement, mob_model, velocity);
        Simulator::Schedule(Seconds(config_moment + flight_duration), &StopNode, mob_model);
        Simulator::Schedule(Seconds(config_moment), &taraAlg, fap);
      }
    }
  }

  MobilityHelper
  configInitPosition(Vector max_box)
  {
    NS_LOG_INFO("INFO: Configuring initial positions...");
    NodeContainer nodes = NodeContainer::GetGlobal();
    MobilityHelper mobility;
    Vector nodepos;
    std::vector<Vector> lst_nodepos;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<UniformRandomVariable> random_num = CreateObject<UniformRandomVariable> ();

    for (uint32_t node = 0; node < nodes.GetN(); node++)
      {
        NS_LOG_DEBUG("DEBUG: Positioning Node: " << node  << " ...");
        if(node == 0) //BKH
          {
            nodepos = Vector (1000, // x
                              1000, // y
                              0); // z
            lst_nodepos.push_back(nodepos);
            positionAlloc->Add(nodepos);
          }
        else if(node == 2) { // Interference node
            // nodepos = Vector(double(RoundIntToMultiple(random_num->GetInteger(0, max_box.x), 1)),
            //                  double(RoundIntToMultiple(random_num->GetInteger(0, max_box.y), 1)),
            //                  0);
            // lst_nodepos.push_back(nodepos);
            // positionAlloc->Add(nodepos);
            nodepos = Vector(960, 1000, 0); // Fixed position
            positionAlloc->Add(nodepos);
            NS_LOG_INFO("Interference node placed at: " << nodepos);
        }
        else if(node == nodes.GetN() - 1) //FGW
          {
            nodepos = GeometricCenter(lst_nodepos);
            positionAlloc->Add(nodepos);
          }
        else
        {
          nodepos = Vector(double(RoundIntToMultiple(random_num->GetInteger(0, max_box.x), 1)),
                           double(RoundIntToMultiple(random_num->GetInteger(0, max_box.y), 1)),
                           0);
          lst_nodepos.push_back(nodepos);
          positionAlloc->Add(nodepos);
        }
        NS_LOG_INFO("INFO: Positioning node: " << node <<" in position: " << nodepos << " ... Ok!");
      }
    mobility.SetPositionAllocator (positionAlloc);

    NS_LOG_INFO("INFO: Configuring initial positions... Ok!");
    return mobility;
  }

  void
  configApps(Ipv4InterfaceContainer interfaces_1, Ipv4InterfaceContainer interfaces_2)
  {
    NS_LOG_INFO ("INFO: Configuring Relay LUPO Apps...");
    ApplicationContainer appContainer;
    NodeContainer c = NodeContainer::GetGlobal();

    PacketSinkHelper rx ("ns3::UdpSocketFactory", InetSocketAddress (interfaces_1.GetAddress(0), 9));
    appContainer.Add(rx.Install (c.Get (0))); //RX - BKH

    OnOffHelper tx ("ns3::UdpSocketFactory", interfaces_2.GetAddress (0)); //TX - FAP
    tx.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
    tx.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
    tx.SetAttribute ("PacketSize", UintegerValue (1400));
    tx.SetAttribute ("DataRate", DataRateValue (DataRate ("70Mbps"))); //above link capacity
    AddressValue remoteAddress (InetSocketAddress (interfaces_1.GetAddress (0), 9)); //Points to RX
    tx.SetAttribute ("Remote", remoteAddress);
    appContainer.Add(tx.Install (c.Get (1))); // TX - FAP
      //Interference (Added)
      OnOffHelper interferenceTx("ns3::UdpSocketFactory", InetSocketAddress(interfaces_1.GetAddress(0), 9)); // BKH's IP
      interferenceTx.SetAttribute("OnTime",  StringValue("ns3::ConstantRandomVariable[Constant=1.0]")); // Always ON
      interferenceTx.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));  // No idle
      interferenceTx.SetAttribute("PacketSize", UintegerValue(2500));
      interferenceTx.SetAttribute("DataRate", DataRateValue(DataRate("100Mbps"))); // High interference

      AddressValue interferenceAddress(InetSocketAddress(interfaces_1.GetAddress(0), 9)); // Same destination
      interferenceTx.SetAttribute("Remote", interferenceAddress);
      appContainer.Add(interferenceTx.Install(c.Get(2))); // TX - INTERFERENCE

      //Packet Sink (Added)
    g_fapIp = interfaces_2.GetAddress(0);  // IP of Node 1 (FAP)

    // For Interferer (Node 2), use its interface
    g_interferenceIp = c.Get(2)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
      PacketSinkHelper interferenceSink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 9)); // Port 9 (matching interference)
      ApplicationContainer sinkApp = interferenceSink.Install(c.Get(0)); // Install on BKH (Node 0)
      sinkApp.Start(Seconds(0.0)); // Start at simulation begin
      sinkApp.Stop(Seconds(100.0)); // Stop at simulation end
    //Debugging purposes
    Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkApp.Get(0));
    if (sink) {
        NS_LOG_UNCOND("Sink exists and is valid");
        sink->TraceConnectWithoutContext("Rx", MakeCallback(&ReceiveInterference));
    } else {
        NS_LOG_UNCOND("ERROR: Sink is null!");
    }
    //



    Ptr<Ipv4> ipv4_rx = c.Get (0)->GetObject<Ipv4> ();
    Ptr<Ipv4> ipv4_tx = c.Get (1)->GetObject<Ipv4> ();
    Ptr<Ipv4> ipv4_relay = c.Get (3)->GetObject<Ipv4> ();

    Ipv4StaticRoutingHelper staticRoutingHelper;

    Ptr<Ipv4StaticRouting> static_routing_rx = staticRoutingHelper.GetStaticRouting(ipv4_rx);
    Ptr<Ipv4StaticRouting> static_routing_tx = staticRoutingHelper.GetStaticRouting (ipv4_tx);

    Ipv4Address ip_address_rx = ipv4_rx->GetAddress (1, 0).GetLocal ();
    Ipv4Address ip_address_tx = ipv4_tx->GetAddress (1, 0).GetLocal ();
    Ipv4Address ip_address_relay = ipv4_relay->GetAddress (1, 0).GetLocal ();
    Ipv4Address ip_address_relay_2 = ipv4_relay->GetAddress (2, 0).GetLocal ();

    static_routing_rx->AddHostRouteTo (ip_address_tx, ip_address_relay, 1);
    static_routing_tx->AddHostRouteTo (ip_address_rx, ip_address_relay_2, 1);

    appContainer.Start (Seconds (0));
    appContainer.Stop(Seconds(100)); // simulation duration

    // FAP → FGW (devices2: index 0 = FAP, index 1 = FGW)
   // bool success =
        Config::ConnectFailSafe("/NodeList/1/DeviceList/0/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&CountTxFAPtoFGW));
    //NS_LOG_UNCOND("Connected CountTxFAPtoFGW: " << (success ? "OK" : "FAILED"));
   //bool success1=
       Config::ConnectFailSafe("/NodeList/3/DeviceList/1/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback(&CountRxFGWfromFAP));
   // NS_LOG_UNCOND("Connected CountTxFAPtoFGW: " << (success1 ? "OK" : "FAILED"));
    // FGW → BKH (devices1: index 1 = FGW, index 0 = BKH)
    //bool success2 =
        Config::ConnectFailSafe("/NodeList/3/DeviceList/0/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&CountTxFGWtoBKH));
    //NS_LOG_UNCOND("Connected CountTxFAPtoFGW: " << (success2 ? "OK" : "FAILED"));
    //bool success3 =
        Config::ConnectFailSafe("/NodeList/0/DeviceList/0/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback(&CountRxBKHfromFGW));
    //NS_LOG_UNCOND("Connected CountTxFAPtoFGW: " << (success3 ? "OK" : "FAILED"));
    // Interference node TX (Node 2)
    //bool success4 =
        Config::ConnectFailSafe("/NodeList/2/DeviceList/0/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&CountTxInterference));

        Config:: ConnectFailSafe("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&CountTotalTx));
        //Config:: Connect("/NodeList/0/DeviceList/0/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",MakeCallback(&SnifferRxCallback));
    Ptr<WifiNetDevice> bkhDevice = DynamicCast<WifiNetDevice>(NodeContainer::GetGlobal().Get(0)->GetDevice(0));
    bkhDevice->GetPhy()->TraceConnectWithoutContext("MonitorSnifferRx", MakeCallback(&SnifferRxCallback));

    //NS_LOG_UNCOND("Connected CountTxFAPtoFGW: " << (success4 ? "OK" : "FAILED"));
    NS_LOG_INFO ("INFO: Configuring Relay LUPO Apps...Ok!");
  }

  WifiHelper
  configWifi (std::string raAlg)
  {
    WifiHelper wifi;
    wifi.SetStandard (WIFI_STANDARD_80211n);

    if (raAlg == "min")
      wifi.SetRemoteStationManager ("ns3::MinstrelHtWifiManager");
    else if (raAlg == "id")
      wifi.SetRemoteStationManager ("ns3::IdealWifiManager");
    else if (raAlg == "lupo")
        wifi.SetRemoteStationManager ("ns3::TARAWifiManager");

    return wifi;
  }

  YansWifiPhyHelper
  configWifiPhy (int freqMHz)
  {
    NS_LOG_INFO ("INFO: Configuring WifiPhy...");
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel;
    wifiPhy.Set ("Frequency", UintegerValue (freqMHz));
    wifiPhy.Set ("ChannelWidth", UintegerValue (20));
    wifiPhy.Set ("RxGain", DoubleValue (0)); // dBi
    wifiPhy.Set ("TxGain", DoubleValue (0)); // dBi
    wifiPhy.Set ("TxPowerStart", DoubleValue (20)); // dBm = 100mW
    wifiPhy.Set ("TxPowerEnd", DoubleValue (20)); // dBm = 100mW
    wifiPhy.SetErrorRateModel ("ns3::NistErrorRateModel");
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss (
          "ns3::FriisPropagationLossModel", "Frequency",
          DoubleValue (freqMHz * 1e6)); // freq in Hz
    wifiPhy.SetChannel (wifiChannel.Create ());

    NS_LOG_INFO ("INFO: Configuring WifiPhy... Ok!");
    return wifiPhy;
  }

  WifiMacHelper
  configWifiMac ()
  {
    NS_LOG_INFO("INFO: Configuring WifiMac...");
    WifiMacHelper wifiMac;
    wifiMac.SetType ("ns3::AdhocWifiMac");
    NS_LOG_INFO ("INFO: Configuring WifiMac... Ok!");
    return wifiMac;
  }

  void
  configMisc ()
  {
    NS_LOG_INFO ("INFO: Configuring other protocols...");

    Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PreambleDetectionModel/"
                "$ns3::ThresholdPreambleDetectionModel/MinimumRssi",
                DoubleValue (-94));

    Config::Set (
        "/NodeList/*/$ns3::Ipv4L3Protocol/InterfaceList/*/ArpCache/AliveTimeout",
        TimeValue (Seconds (500)));
    Config::Set (
        "/NodeList/*/$ns3::ArpL3Protocol/CacheList/*/AliveTimeout",
        TimeValue (Seconds (500)));
    Config::Set (
        "/NodeList/*/$ns3::Ipv4L3Protocol/InterfaceList/*/ArpCache/DeadTimeout",
        TimeValue (Seconds (500)));
    Config::Set (
        "/NodeList/*/$ns3::ArpL3Protocol/CacheList/*/DeadTimeout",
        TimeValue (Seconds (500)));

    NS_LOG_INFO ("INFO: Configuring other protocols... Ok!");
  }

} // namespace ns3