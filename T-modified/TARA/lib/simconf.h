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

#include <ns3/wifi-module.h>
#include <ns3/mobility-module.h>
#include <ns3/internet-module.h>
//Added by Daniel
#include <fstream>
#include <string>
//End of Add
namespace ns3
{
  void configNodeMobility();

  MobilityHelper configInitPosition(Vector max_box);

  void configApps(Ipv4InterfaceContainer interfaces_1, Ipv4InterfaceContainer interfaces_2);

  WifiHelper configWifi(std::string raAlg);

  YansWifiPhyHelper configWifiPhy(int freqMHz);

  WifiMacHelper configWifiMac();

  void configMisc();
//Added Daniel
extern uint64_t txPacketsFAPtoFGW;
extern uint64_t rxPacketsFGWfromFAP;

extern uint64_t txPacketsFGWtoBKH;
extern uint64_t rxPacketsBKHfromFGW;

extern uint64_t txPacketsInterferer;
extern uint64_t totaltxPackets;
extern double currentSnrBkh;

//

 void PrintDeviceSummary();

} // namespace ns3