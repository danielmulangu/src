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

#include <string>
#include "ns3/core-module.h"
#include <ns3/mobility-module.h>
#include <ns3/vector.h>

namespace ns3
{

struct NodeMovInfo 
{
  Vector current_pos;
  Vector future_pos;
  Vector velocity = Vector (0,0,0);
  double flight_duration = 0;
};

void taraAlg(struct NodeMovInfo fap);
struct NodeMovInfo CalcFGWmov(struct NodeMovInfo fgw);
Vector CalcFuturePosition(struct NodeMovInfo node, double t);
double PredictSNR(double time, struct NodeMovInfo node1, struct NodeMovInfo node2);
void ConfigNewTARASnr(double new_snr, std::string node_id, std::string device_id);
void SetNodeMovement(Ptr<MobilityModel> mobmodel, Vector velocity);
void StopNode(Ptr<MobilityModel> mob_model);
Vector GeometricCenter(std::vector<Vector> positions);
double CalcPathLossComponent(double distance, double ch_frequency);
double receivedPower(double path_loss, int tx_power, int tx_gain, int rx_gain);
int RoundIntToMultiple(int number, int multiple);

} // namespace ns3