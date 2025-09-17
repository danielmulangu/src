#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/flow-monitor-module.h"
#include <iomanip>
#include <vector>
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiManagerComparison");

// Structure to store simulation results
struct SimulationResult {
    std::string wifiManager;
    std::vector<double> timestamps;
    std::vector<Vector> node1Positions;
    std::vector<Vector> node2Positions;
    std::vector<double> distances;
    std::vector<double> throughputs;
};

void TrackPositionsAndDistance(Ptr<Node> node1, Ptr<Node> node2,
                              SimulationResult& result, double time) {
    Ptr<MobilityModel> mobility1 = node1->GetObject<MobilityModel>();
    Ptr<MobilityModel> mobility2 = node2->GetObject<MobilityModel>();

    Vector pos1 = mobility1->GetPosition();
    Vector pos2 = mobility2->GetPosition();

    double distance = std::sqrt(std::pow(pos1.x - pos2.x, 2) +
                               std::pow(pos1.y - pos2.y, 2) +
                               std::pow(pos1.z - pos2.z, 2));

    result.timestamps.push_back(time);
    result.node1Positions.push_back(pos1);
    result.node2Positions.push_back(pos2);
    result.distances.push_back(distance);
}

void PrintResults(const SimulationResult& result) {
    std::cout << "\n=== Results for " << result.wifiManager << " ===\n";
    std::cout << std::setw(8) << "Time(s)"
              << std::setw(15) << "Node1 Pos"
              << std::setw(15) << "Node2 Pos"
              << std::setw(12) << "Distance(m)"
              << std::setw(15) << "Throughput(Mbps)\n";
    std::cout << std::string(65, '-') << "\n";

    for (size_t i = 0; i < result.timestamps.size(); ++i) {
        std::cout << std::setw(8) << std::fixed << std::setprecision(1) << result.timestamps[i]
                  << std::setw(8) << "(" << std::setprecision(2) << result.node1Positions[i].x
                  << "," << result.node1Positions[i].y << ")"
                  << std::setw(8) << "(" << result.node2Positions[i].x
                  << "," << result.node2Positions[i].y << ")"
                  << std::setw(12) << std::setprecision(2) << result.distances[i]
                  << std::setw(15) << std::setprecision(3) << result.throughputs[i] << "\n";
    }
}

SimulationResult RunSimulation(const std::string& wifiManager, uint32_t runNumber) {
    NS_LOG_INFO("Running simulation with " << wifiManager << " (Run " << runNumber << ")");

    SimulationResult result;
    result.wifiManager = wifiManager;

    // Create nodes
    NodeContainer nodes;
    nodes.Create(2);

    // Set up WiFi
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    // Set the WiFi manager
    wifi.SetRemoteStationManager("ns3::" + wifiManager + "WifiManager");

    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    // Set up mobility
    MobilityHelper mobility;

    // Node 0 - static at position (0, 0, 0)
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    positionAlloc->Add(Vector(5.0, 0.0, 0.0)); // Start close to node 0

    mobility.SetPositionAllocator(positionAlloc);

    // Node 1 moves away from node 0 at constant velocity (1 m/s in x-direction)
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(nodes);

    Ptr<ConstantVelocityMobilityModel> mobileNode =
        nodes.Get(1)->GetObject<ConstantVelocityMobilityModel>();
    mobileNode->SetVelocity(Vector(1.0, 0.0, 0.0)); // Move at 1 m/s in x-direction

    // Install internet stack
    InternetStackHelper internet;
    internet.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    // Set up applications
    uint16_t port = 9;

    // UDP server on node 0
    PacketSinkHelper sink("ns3::UdpSocketFactory",
                         InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer serverApp = sink.Install(nodes.Get(0));
    serverApp.Start(Seconds(0.0));
    serverApp.Stop(Seconds(60.0));

    // UDP client on node 1
    OnOffHelper client("ns3::UdpSocketFactory",
                      InetSocketAddress(interfaces.GetAddress(0), port));
    client.SetConstantRate(DataRate("100Mbps"), 1024);
    ApplicationContainer clientApp = client.Install(nodes.Get(1));
    clientApp.Start(Seconds(1.0));
    clientApp.Stop(Seconds(59.0));

    // Install flow monitor
    FlowMonitorHelper flowMonitor;
    Ptr<FlowMonitor> monitor = flowMonitor.InstallAll();

    // Schedule position tracking every second
    for (double time = 0.0; time <= 60.0; time += 1.0) {
        Simulator::Schedule(Seconds(time), &TrackPositionsAndDistance,
                          nodes.Get(0), nodes.Get(1), std::ref(result), time);
    }

    // Run simulation
    Simulator::Stop(Seconds(60.0));
    Simulator::Run();

    // Calculate throughput for each 1-second window
    Ptr<FlowMonitor> flowMon = flowMonitor.GetMonitor();
    FlowMonitor::FlowStatsContainer stats = flowMon->GetFlowStats();

    // Initialize throughput vector with zeros for each second
    result.throughputs.resize(60, 0.0);

    for (auto& stat : stats) {
        // Get the total bytes received and time of last packet
        uint64_t totalBytes = stat.second.rxBytes;
        double totalTime = stat.second.timeLastRxPacket.GetSeconds() - stat.second.timeFirstTxPacket.GetSeconds();

        if (totalTime > 0) {
            // Calculate average throughput for the entire simulation
            double avgThroughput = (totalBytes * 8.0) / (totalTime * 1000000.0); // Mbps

            // For simplicity, we'll use the average throughput for each second
            // In a more complex implementation, you'd need to track packet timestamps
            for (size_t i = 0; i < result.throughputs.size(); ++i) {
                if (i + 1 <= totalTime) { // Only for seconds when traffic was active
                    result.throughputs[i] = avgThroughput;
                }
            }
        }
    }

    Simulator::Destroy();

    return result;
}

int main(int argc, char *argv[]) {
    // Enable logging
    // LogComponentEnable("WifiManagerComparison", LOG_LEVEL_INFO);

    std::vector<std::string> wifiManagers = {"MinstrelHt", "ThompsonSampling", "Ideal"};
    std::vector<SimulationResult> results;

    // Run simulation for each WiFi manager
    for (uint32_t i = 0; i < wifiManagers.size(); ++i) {
        SimulationResult result = RunSimulation(wifiManagers[i], i + 1);
        results.push_back(result);
        PrintResults(result);
    }

    // Summary comparison
    std::cout << "\n=== Summary Comparison ===\n";
    std::cout << std::setw(15) << "WifiManager"
              << std::setw(15) << "Avg Throughput"
              << std::setw(15) << "Max Throughput"
              << std::setw(15) << "Min Throughput\n";
    std::cout << std::string(60, '-') << "\n";

    for (const auto& result : results) {
        double avg = 0.0, max = 0.0, min = std::numeric_limits<double>::max();
        for (double throughput : result.throughputs) {
            avg += throughput;
            if (throughput > max) max = throughput;
            if (throughput < min && throughput > 0) min = throughput;
        }
        avg /= result.throughputs.size();

        std::cout << std::setw(15) << result.wifiManager
                  << std::setw(15) << std::setprecision(3) << avg
                  << std::setw(15) << max
                  << std::setw(15) << (min == std::numeric_limits<double>::max() ? 0 : min) << "\n";
    }

    return 0;
}