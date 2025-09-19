/*The code is testing and comparing the Thompson Sampling algorithm with other algorithms (Minstrel
 * and Ideal)*/

#include "../src/mobility/model/constant-position-mobility-model.h"
#include "../src/mobility/model/position-allocator.h"

#include <ns3/core-module.h>
#include <ns3/internet-module.h>
#include <ns3/network-module.h>
#include <ns3/wifi-module.h>
#include <ns3/mobility-module.h>
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Daniel");

int main(int argc, char *argv[])
{
    std::vector<std::string> wifiManagers = {"MinstrelHt", "ThompsonSampling", "Ideal"};
    NodeContainer ap_nodes;
    ap_nodes.Create(1);

    NodeContainer sta_nodes;
    sta_nodes.Create(1);

    YansWifiPhyHelper wifiphy;
    YansWifiChannelHelper wifi_channel = YansWifiChannelHelper::Default();

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);

    WifiMacHelper  wifi_mac;
    wifi_mac.SetType("ns3""AdhocWifiMac");

    wifiphy.SetChannel(wifi_channel.Create());

    wifi.SetRemoteStationManager(("ns3::" + wifiManger + "wifiManager").c_str());

    NetDeviceContainer ap_dev = wifi.Install(wifiphy, wifi_mac, ap_nodes);
    NetDeviceContainer sta_dev = wifi.Install(wifiphy, wifi_mac, sta_nodes);

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> ap_pos_allocator = CreateObject<ListPositionAllocator> ();
    ap_pos_allocator->Add(Vector (0.0, 0.0, 0.0));
    mobility.SetPositionAllocator(ap_pos_allocator);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(ap_nodes);

    Ptr<ListPositionAllocator> sta_pos_allocator = CreateObject<ListPositionAllocator> ();
    sta_pos_allocator->Add(Vector (5.0, 0.0, 0.0));
    mobility.SetPositionAllocator(sta_pos_allocator);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(sta_nodes);

    Ptr<ConstantVelocityMobilityModel> v_mob = sta_dev.Get(0)->GetObject<ConstantVelocityMobilityModel> ();
    v_mob->SetVelocity(Vector (1.0, 0.0, 0.0));

    //Internet Stack
    InternetStackHelper internet;
    internet.Install(ap_nodes);
    internet.Install(sta_nodes);

    Ipv4AddressHelper ipv4_address;
    ipv4_address.SetBase("10.1.1.0, 255.255.255.0");
    Ipv4InterfaceContainer ipv4_interface = ipv4_address.Assign(ap_dev);
    Ipv4InterfaceContainer ipv4_interface2 = ipv4_address.Assign(sta_dev);

    





}
