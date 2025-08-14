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


#include "simlogs.h"

#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/constant-velocity-mobility-model.h>
#include <ns3/node.h>
#include <ns3/node-container.h>
#include <ns3/vector.h>
#include "ns3/core-module.h"


// New additions needed for enhanced monitoring:
#include "tara.h"
#include <ns3/packet-sink.h>          // For PacketSink class
#include <ns3/application-container.h> // For ApplicationContainer
#include <ns3/ipv4-static-routing.h>   // For routing table inspection
#include <ns3/ipv4-routing-helper.h>   // For routing helper
#include <ns3/inet-socket-address.h>   // For address conversion
#include <ns3/packet.h>
#include <ns3/simulator.h>
#include <ns3/config.h>

std::vector <uint32_t> rxByteCounter = {};
std::vector <uint32_t> oldRxByteCounter = {};
std::map<int, std::array <uint32_t, 2>> traceIds;

std::ofstream throughputLog; //file where throughput log is output
std::ofstream positionsLog; //file where positions log is output
std::ofstream distancesLog; //file where distances log is output


namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("simlogs");

  void
  Monitor (bool firstTime)
  {

    double frequency = 1; //loops every 1 second    
    static int tracesConnected=0;
    double now = Simulator::Now ().GetSeconds ();
    NS_LOG_INFO("SimTime: " << now);

    if(firstTime)
    {
      //Throughput log
      throughputLog.open("throughput.csv", std::ios_base::out | std::ios_base::app);

      throughputLog << "SimTime"; //Header

      for (uint32_t nodeId = 0; nodeId < NodeContainer::GetGlobal().GetN(); nodeId++)
      {
        for (uint32_t devId = 0; devId < NodeContainer::GetGlobal().Get(nodeId)->GetNDevices(); devId++)
        {
          bool success = Config::ConnectFailSafe("/NodeList/" + std::to_string(nodeId) + "/DeviceList/" + std::to_string(devId) + "/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback(&ReceivePacket));
          if(success)
          {
            std::array<uint32_t, 2> idArray = {nodeId, devId};
            traceIds.insert(std::pair<int, std::array<uint32_t, 2>>(tracesConnected, idArray));
            tracesConnected++;
            throughputLog << ";node-" << nodeId << "_dev-" << devId;
          }
        }
      }
      throughputLog << std::endl;
      rxByteCounter.resize(tracesConnected + 1, 0);
      oldRxByteCounter.resize(tracesConnected + 1, 0);
      
      //positions Log
      positionsLog.open("positions.csv", std::ios_base::out | std::ios_base::app);

      positionsLog << "SimTime";
      for (uint32_t i = 0; i <= NodeContainer::GetGlobal().GetN(); i++)
        positionsLog << ";n" << i << "x;"
                        << "n" << i << "y;"
                        << "n" << i << "z";

      positionsLog << std::endl;
      distancesLog.open("distances.csv", std::ios_base::out | std::ios_base::app);
      distancesLog << "SimTime";
      for (uint32_t i = 0; i < 4; i++)
        distancesLog << ";n" << i << "_n" << 2;

      distancesLog << std::endl;  
    }

    throughputLog << now;
    for (int counter = 0; counter < tracesConnected; counter++)
    {
      throughputLog << ";" << Throughputs(rxByteCounter.at(counter), oldRxByteCounter.at(counter), frequency);
      //consoleLog | Mbit/s only valid if frequency=1
      NS_LOG_INFO("Trace: " << counter << " | Throughput (Mbit/s): " << Throughputs(rxByteCounter.at(counter), oldRxByteCounter.at(counter), frequency));  
    }
    throughputLog << std::endl;

    positionsLog << now << Positions() << std::endl;
    distancesLog << now << Distances() << std::endl;
    
    oldRxByteCounter = rxByteCounter;
      // Add these NEW interference monitoring lines:
    static uint32_t lastRxCount = 0;
      Ptr<PacketSink> sink = DynamicCast<PacketSink>(NodeContainer::GetGlobal().Get(0)->GetApplication(0));
      uint32_t currentRx = sink->GetTotalRx();
      uint32_t newPackets = currentRx - lastRxCount;
      lastRxCount = currentRx;
      //NS_LOG_UNCOND("Interference packets at BKH: " << sink->GetTotalRx());
      //NS_LOG_UNCOND("Interference pkt/s: " << newPackets << " (Total: " << currentRx << ")");
    // NS_LOG_UNCOND("Total Rx Byte sent/s: Trace 0: "
    //         << rxByteCounter[0]
    //         << ", Trace 4: "
    //         << rxByteCounter[4]
    //         << " (From previous window: Trace 0: "
    //         << oldRxByteCounter[0]
    //         << ", Trace 4: "
    //         << oldRxByteCounter[4]
    //         << ")");


      Vector fgwPos = NodeContainer::GetGlobal().Get(3)->GetObject<MobilityModel>()->GetPosition();
      Vector bkhPos = NodeContainer::GetGlobal().Get(0)->GetObject<MobilityModel>()->GetPosition();
      Vector interf = NodeContainer::GetGlobal().Get(2)->GetObject<MobilityModel>()->GetPosition();
      // Create temporary structs
      NodeMovInfo fgwNode, bkhNode, interfNode;
      fgwNode.current_pos = fgwPos;
      bkhNode.current_pos = bkhPos;
      interfNode.current_pos = interf;
      double nowMs = Simulator::Now().GetSeconds() * 1000.0;
      double distance = CalculateDistance(fgwPos, bkhPos);
      double snr = PredictSNR(nowMs, fgwNode, bkhNode); // Use for instant SNR
      NS_LOG_UNCOND(" Predictive Current BKH SNR: " << snr << " dB (Distance (FGW/BKH): " << distance << "m) @" << Simulator::Now().GetSeconds() << " s");
      double psinr = PredictSNR(nowMs, interfNode, bkhNode);
      NS_LOG_UNCOND("\n Predictive SNR with Intef and BKH" << psinr << " dB (Distance (FGW/BKH): " << distance << "m) @" << Simulator::Now().GetSeconds() << " s");
    Simulator::Schedule(Seconds (frequency), &Monitor, false);
  }

  void
  ReceivePacket (std::string context, Ptr<const Packet> packet)
  {
    uint32_t *result = ParseContextString(context);
    std::array<uint32_t, 2> ids = {result[0], result[1]};

    for(auto i : traceIds)
    {
      if(i.second == ids)
      {
        rxByteCounter[i.first] += packet->GetSize ();
        break;
      }
    }
  }

  uint32_t* 
  ParseContextString(std::string context) 
  {
    uint32_t* numbers = new uint32_t[2];
    int count = 0;
    std::string number;

    for (char c : context)
    {
      if (std::isdigit(c))
          number += c;
      else if (!number.empty())
      {
          numbers[count++] = std::stoi(number);
          number.clear();
          if (count == 2)
              break; //Only nodelist Id and Applicationlist Id is needed 
      }
    } 
    return numbers;
  }

  std::string
  Positions()
  {
    std::stringstream ss;
    NodeContainer c = NodeContainer::GetGlobal();
    
    for (uint32_t i = 0; i < c.GetN(); i++) //log nodes position
    {
      Ptr<MobilityModel> mobility = c.Get(i)->GetObject<MobilityModel>();
      Vector pos = mobility->GetPosition();
      ss << ";" << pos.x << ";" << pos.y << ";" << pos.z;
      NS_LOG_DEBUG("Node "<< i << " position: \t" << pos.x << "\t" << pos.y << "\t" << pos.z);
    }
    return ss.str();
  }

  std::string
  Distances()
  {
    std::stringstream ss;
    NodeContainer c = NodeContainer::GetGlobal();
   
    //FGW is assumed to be the last node id
    Vector fgwpos = c.Get(c.GetN() - 1)->GetObject<MobilityModel>()->GetPosition();
    for (uint32_t i = 0; i < c.GetN() - 1; i++) // log distances between FGW and other nodes
    {
      Vector nodepos = c.Get(i)->GetObject<MobilityModel>()->GetPosition();
      double dist = CalculateDistance(fgwpos, nodepos);
      ss << ";" << dist;
      NS_LOG_DEBUG("Node "<< i << " distance to FGW: \t" << dist);
    }
    
    return ss.str();    
  }

  std::string
  Throughputs(double rxByteCounter, double oldRxByteCounter, double frequency)
  {
    std::stringstream ss;
    
    double throughput = ((rxByteCounter - oldRxByteCounter) * 8) / (1e6 * frequency);

    ss << throughput;

    return ss.str();
  }

  void
  configLogs ()
  {
    LogComponentEnable("sim", LOG_INFO);
    LogComponentEnable("simlogs", LOG_INFO);
    LogComponentEnable("simconf", LOG_INFO);
    LogComponentEnable("tarafuncs", LOG_INFO);

    //



    //
  }



} // namespace ns3