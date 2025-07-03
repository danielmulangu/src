#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/point-to-point-helper.h"
#include <ns3/antenna-module.h>
#include <cmath>
#include <chrono>

using namespace ns3;

//Position
static Ptr<ListPositionAllocator>
GetGnbPositions(double gNbHeight)
{
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0, 0.0, gNbHeight));

    return pos;
}

static Ptr<ListPositionAllocator>
GetUePositions(double ueY, double ueHeight)
{
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    /*For curiosity purposes, let's set the UE far away from the Gnb and Check how would the CQI and MCS behaves*/

    pos->Add(Vector(0.0, ueY, ueHeight));
    std::cout<<"Successfully installed positions!"<<std::endl;
    return pos;
}


static std::vector<uint64_t> packetsTime;

static Vector GetFinalPosition (Ptr <Node> simNode)
{
    Ptr<MobilityModel> mobility = simNode->GetObject<MobilityModel> ();
    if (!mobility)
    {
        NS_FATAL_ERROR("Node has no mobility");
    }
    return mobility->GetPosition();
}
//Printing the Nodes positions (For debugging purposes, again)
static void PrintPositions(NodeContainer sim_node)
{
    for (uint32_t i=0; i< sim_node.GetN();++i)
    {
        Vector position_s = GetFinalPosition(sim_node.Get(i));
        std::cout <<"Position (X,Y,Z)" << position_s << std::endl;
    }
}

int
main (int argc, char *argv[])
{
    double ueY = 20.0;
    double gNbHeight = 10.0;
    double ueHeight = 20.0;
    const uint8_t gNbNum = 1;
    const uint8_t ueNum = 1;
    double simDuration = 30.0; // seconds

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time", simDuration);
    cmd.AddValue("ueY", "Y position of any UE", ueY);

    NodeContainer gnbNodes; gnbNodes.Create (gNbNum);
    NodeContainer ueNodes;  ueNodes.Create (ueNum);

    //Position: Set the positions of the nodes using the height and coordinates of the two nodes.
    Ptr<ListPositionAllocator> gNbPositionAlloc = GetGnbPositions(gNbHeight);
    Ptr<ListPositionAllocator> uePositionAlloc = GetUePositions(ueY, ueHeight);

    // Mobility
    MobilityHelper mobility;
    MobilityHelper uemob;
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(gNbPositionAlloc);
    mobility.Install (gnbNodes);
    uemob.SetMobilityModel ("ns3::ConstantVelocityMobilityModel"); // Set the model  for ueNodel
    uemob.SetPositionAllocator (uePositionAlloc);
    uemob.Install (ueNodes);
    std::cout <<"UE initial position:"<<std::endl;
    PrintPositions(ueNodes);
    std:: cout<<"GNb initial position:"<<std::endl;
    PrintPositions(gnbNodes);
    for (uint n = 0; n < ueNodes.GetN (); n++)
    {
        Ptr<ConstantVelocityMobilityModel> mob = ueNodes.Get(n)->GetObject<ConstantVelocityMobilityModel>();
        // Set the desired velocity
        mob->SetVelocity(Vector(1.0, 5.0, 0.0)); // Moving at 1 m/s in the x direction and 5 m/s in the y direction
    }
    std::cout <<"UE initial position:"<<std::endl;
    PrintPositions(ueNodes);
    std:: cout<<"GNb initial position:"<<std::endl;
    PrintPositions(gnbNodes);

    return 0;

}