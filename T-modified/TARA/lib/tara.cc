
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

#include "tara.h"
#include "simconf.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include <ns3/internet-module.h>
#include "ns3/log.h"
#include <ns3/node.h>
#include <ns3/node-container.h>

namespace ns3
{

  NS_LOG_COMPONENT_DEFINE("tarafuncs");

  void
  taraAlg(struct NodeMovInfo fap)
  {
    NodeContainer c = NodeContainer::GetGlobal();
    uint8_t new_interval = 30;
    struct NodeMovInfo backhaul, fgw;

    backhaul.current_pos = c.Get(0)->GetObject<MobilityModel>()->GetPosition();
    fgw.current_pos = c.Get(c.GetN()-1)->GetObject<MobilityModel>()->GetPosition();
    
    // ----------------- Geometric Center -----------------
    std::vector<Vector> nodepos{fap.future_pos, backhaul.current_pos}; //FAP + BKH
    fgw.future_pos = GeometricCenter(nodepos);
    // ----------------- Geometric Center -----------------

    fgw = CalcFGWmov(fgw); // !!!! assuming FGW is always the last ID of the container !!!!

    //Predict Future SNR of each TARAstation
    for (double tf = 0; tf < new_interval*1000; tf+=500)
    {
      double future_snr_2 = PredictSNR(tf, fgw, backhaul);
      Simulator::Schedule(MilliSeconds(tf), &ConfigNewTARASnr, future_snr_2, "3", "0"); //fgw: Changed to 3 because of the assumption that FGW is always the last node
                                                                                                  //and i added the interference
      double future_snr_1 = PredictSNR(tf, fgw, fap);
      Simulator::Schedule(MilliSeconds(tf), &ConfigNewTARASnr, future_snr_1, "1", "0"); //fap
    }
  }

  void ConfigNewTARASnr(double new_snr, std::string node_id, std::string device_id)
  {
      std::string config_string = "/NodeList/" + node_id + "/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$ns3::TARAWifiManager/TARASnr"; //changes tara-wifi-manager.cc
      Config::Set(config_string, DoubleValue(new_snr));
  }

  struct NodeMovInfo CalcFGWmov(struct NodeMovInfo fgw)
  { 
    int fixed_velocity = 8; // -> fixed velocity ~30km/h -> 8m/s
    NodeContainer c = NodeContainer::GetGlobal();
    Ptr<ConstantVelocityMobilityModel> mob_model =
      DynamicCast<ConstantVelocityMobilityModel> (c.Get(c.GetN()-1)->GetObject<MobilityModel> ());
      
    int dx = fgw.future_pos.x - fgw.current_pos.x;
    int dy = fgw.future_pos.y - fgw.current_pos.y;
    int dz = fgw.future_pos.z - fgw.current_pos.z;
    
    fgw.flight_duration = sqrt((pow(abs(dx),2) + pow(abs(dy),2) + pow(abs(dz),2)) / pow(fixed_velocity, 2)); 
    
    if(fgw.flight_duration > 0)
    {
      fgw.velocity.x = dx / fgw.flight_duration;
      fgw.velocity.y = dy / fgw.flight_duration;
      fgw.velocity.z = dz / fgw.flight_duration;
    }
    else
    {
      //if it doesn't fly, the velocity is 0.
      fgw.velocity.x = 0;
      fgw.velocity.y = 0;
      fgw.velocity.z = 0;
    }
    
    mob_model->SetVelocity(fgw.velocity);
    Simulator::Schedule(Seconds(fgw.flight_duration), &StopNode, mob_model);

    return fgw;
  }

  Vector CalcFuturePosition(struct NodeMovInfo node, double t)
  {
    if(t < node.flight_duration)
      return Vector (node.current_pos.x+node.velocity.x*t ,
                    node.current_pos.y+node.velocity.y*t,
                    node.current_pos.z+node.velocity.z*t);
    else
      return Vector (node.current_pos.x+node.velocity.x*node.flight_duration,
                    node.current_pos.y+node.velocity.y*node.flight_duration,
                    node.current_pos.z+node.velocity.z*node.flight_duration);
  }

  double PredictSNR(double time, struct NodeMovInfo node1, struct NodeMovInfo node2)
  {
    double noise_power = 3.16e-13;
    double ch_frequency=5180000000;
    int tx_power=20, tx_gain=0, rx_gain=0;
    double timeSeconds = time / 1000;

    double distance = CalculateDistance(CalcFuturePosition(node1, timeSeconds), CalcFuturePosition(node2, timeSeconds));
    NS_ABORT_MSG_IF(distance == 0, "ERROR: Distance between nodes cannot be 0.");

    double snrval = 10*log10(receivedPower(CalcPathLossComponent(distance, ch_frequency), 
                              tx_power, tx_gain, rx_gain)/noise_power);

    return snrval;
  }

  void SetNodeMovement(Ptr<MobilityModel> mobmodel, Vector velocity)
  {
    Ptr<ConstantVelocityMobilityModel> mob_model = DynamicCast<ConstantVelocityMobilityModel> (mobmodel);
    mob_model->SetVelocity(velocity);
  }

  void StopNode(Ptr<MobilityModel> mobmodel)
  {
    Ptr<ConstantVelocityMobilityModel> mob_model = DynamicCast<ConstantVelocityMobilityModel> (mobmodel);

    mob_model->SetVelocity(Vector (0,0,0)); //stop node movement

    //WARNING: Fix to the Problem of not round values
    Vector pos = mob_model->GetPosition();
    pos.x = RoundIntToMultiple(pos.x, 1);
    pos.y = RoundIntToMultiple(pos.y, 1);
    pos.z = RoundIntToMultiple(pos.z, 1);
    mob_model->SetPosition(pos);
  }

  Vector
  GeometricCenter(std::vector <Vector> positions)
  {
    double sumX=0, sumY=0, sumZ=0;
    int size = positions.size();

    for (Vector pos : positions)
    {
        sumX += pos.x;
        sumY += pos.y;
        sumZ += pos.z;
    }
    return Vector(sumX / size, sumY / size, sumZ / size);
  }

  double
  CalcPathLossComponent(double distance, double ch_frequency)
  {
    // Calculates the path loss component using the Friis transmission equation
    return (20 * log10(distance) + 20*log10(ch_frequency) + 20*log10(4*M_PI/3e8));
  }

  double 
  receivedPower(double path_loss, int tx_power, int tx_gain, int rx_gain)
  {
    // Calculates the received power using the Friis transmission equation
    return (pow(10,(tx_power + tx_gain + rx_gain - path_loss)/10)/1000);
  }

  int
  RoundIntToMultiple(int number, int multiple)
  {
    if(number % multiple >= multiple/2)
      return ((number/multiple) + 1)*multiple;
    else
      return (number/multiple)*multiple;
  }


}