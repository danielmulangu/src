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

#include "lib/simlogs.h"
#include "lib/simconf.h"
#include "lib/tara.h"
#include <ns3/network-module.h>
#include <ns3/wifi-module.h>
#include <ns3/internet-module.h>
#include <ns3/core-module.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("sim");
void PrintPacketStats()
{
    std::cout << "[Time: " << Simulator::Now().GetSeconds() << "s]\n";
    std::cout << "  FAP → FGW: TX = " << txPacketsFAPtoFGW
              << ", RX = " << rxPacketsFGWfromFAP << "\n";
    std::cout << "  FGW → BKH: TX = " << txPacketsFGWtoBKH
              << ", RX = " << rxPacketsBKHfromFGW << "\n";
    std::cout << "  Interference Node TX = " << txPacketsInterferer << "\n";
    std:: cout <<"  Total TX = "<<totaltxPackets <<"\n";
    std:: cout << " Actual SNR at BKH: "<<currentSnrBkh <<"\n\n";

    // Reschedule the next print
    Simulator::Schedule(Seconds(1.0), &PrintPacketStats);
}
int
main (int argc, char **argv)
{
  configLogs();
  double simSeed=10;
  std::string raAlg = "tara";

  CommandLine cmd; 
  cmd.AddValue ("simSeed", "random generator seed", simSeed);
  cmd.AddValue ("raAlg", "tara, min, id", raAlg);
  cmd.Parse (argc, argv);  

  RngSeedManager::SetSeed (simSeed);
  RngSeedManager::SetRun (2);

  NodeContainer adhocNodes;
  adhocNodes.Create(4); // NODE 0 = BKH ; NODE 1 = FAP ;  NODE 2 = FGW; NODE 3 = Interference(Added by Daniel)

  configNodeMobility(); //aqui
 
  WifiHelper wifi = configWifi (raAlg);
  
  YansWifiPhyHelper wifiPhy1, wifiPhy2;
  
  wifiPhy1 = configWifiPhy(5180);
  wifiPhy2 = configWifiPhy(5240);

  WifiMacHelper wifiMac = configWifiMac ();

  NetDeviceContainer devices1, devices2;

  devices1.Add(wifi.Install (wifiPhy1, wifiMac, adhocNodes.Get(0))); //bkh
  devices1.Add(wifi.Install (wifiPhy1, wifiMac, adhocNodes.Get(3))); //fgw0

  //Interferer
  devices1.Add(wifi.Install (wifiPhy1, wifiMac, adhocNodes.Get(2))); //interferer


  devices2.Add(wifi.Install (wifiPhy2, wifiMac, adhocNodes.Get(1))); //fap
  devices2.Add(wifi.Install (wifiPhy2, wifiMac, adhocNodes.Get(3))); //fgw1
    
  InternetStackHelper internet;
  internet.Install (adhocNodes);

  Ipv4AddressHelper ipv4_1, ipv4_2;
  ipv4_1.SetBase ("10.0.0.0", "255.255.255.0");
  ipv4_2.SetBase ("11.0.0.0", "255.255.255.0");
  
  Ipv4InterfaceContainer interfaces_1, interfaces_2;
  interfaces_1 = ipv4_1.Assign (devices1);
  interfaces_2 = ipv4_2.Assign (devices2);

  configApps (interfaces_1, interfaces_2);
  configMisc ();

  Monitor(true);
    PrintDeviceSummary();
    Simulator::Schedule(Seconds(1.0), &PrintPacketStats);
    Simulator::Stop (Seconds (100));
    Simulator::Run ();
    Simulator::Destroy ();

  return 0;
}
