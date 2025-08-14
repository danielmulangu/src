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

#pragma once

#include <string>
#include <ns3/wifi-module.h>
#include <ns3/core-module.h>


namespace ns3
{
 
  void Monitor(bool firstTime);

  void ReceivePacket(std::string context, Ptr<const Packet> packet);

  uint32_t* ParseContextString(std::string context);


  /**
  * @brief Outputs the nodes positions, sorted by ID.
  * @return A String containing every node position in a .csv format
  */
  std::string Positions();

  /**
  * @brief Outputs the distance between each node ID and the FGW, sorted by ID.
  * @warning The FGW is assumed to be the last node ID
  * @return A String containing every node distance to the FGW in a .csv format
  */
  std::string Distances();

  /**
  * @brief Outputs the link throughputs, sorted by receiver ID.
  * @return A String containing every node position in a .csv format
  */
  std::string Throughputs(double rxByteCounter, double oldRxByteCounter, double frequency);

  void configLogs();

} // namespace ns3