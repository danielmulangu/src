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
#include <fstream>
#include <sstream>
#include <limits>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiManagerComparison");

static const int SIM_DURATION = 60;  // seconds

// Structure to store simulation results
struct SimulationResult {
    std::string wifiManager;
    std::vector<double> timestamps;
    std::vector<Vector> node1Positions;
    std::vector<Vector> node2Positions;
    std::vector<double> distances;
    std::vector<double> throughputs;       // Mbps per second
    std::vector<uint32_t> rxPacketsPerSec; // per-second deltas
    std::vector<uint32_t> lostPacketsPerSec; // per-second deltas
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

static void PrintResultsToTerminal(const SimulationResult& result) {
    std::cout << "\n=== Results for " << result.wifiManager << " ===\n";
    std::cout << "Time(s)  Node0(x,y)   Node1(x,y)   Dist(m)    Mbps     RxPkts  LostPkts\n";
    std::cout << std::string(78, '-') << "\n";

    for (size_t i = 0; i < result.throughputs.size(); ++i) {
        // guard for vectors of equal length
        Vector p0 = (i < result.node1Positions.size()) ? result.node1Positions[i] : Vector();
        Vector p1 = (i < result.node2Positions.size()) ? result.node2Positions[i] : Vector();
        double dist = (i < result.distances.size()) ? result.distances[i] : 0.0;

        std::cout << std::setw(6) << (i + 1)
                  << std::setw(5) << "  (" << std::fixed << std::setprecision(2) << p0.x
                  << "," << p0.y << ")"
                  << std::setw(6) << "  (" << p1.x << "," << p1.y << ")"
                  << std::setw(10) << std::setprecision(2) << dist
                  << std::setw(10) << std::setprecision(3) << result.throughputs[i]
                  << std::setw(9)  << (i < result.rxPacketsPerSec.size() ? result.rxPacketsPerSec[i] : 0)
                  << std::setw(10) << (i < result.lostPacketsPerSec.size() ? result.lostPacketsPerSec[i] : 0)
                  << "\n";
    }
}

static void SaveResultsToFile(const SimulationResult& result) {
    std::ostringstream fname;
    fname << "results_" << result.wifiManager << ".txt";
    std::ofstream out(fname.str());

    out << "WiFiManager: " << result.wifiManager << "\n";
    out << "Time(s),Node0_x,Node0_y,Node1_x,Node1_y,Distance(m),Throughput(Mbps),RxPkts,LostPkts\n";

    for (size_t i = 0; i < result.throughputs.size(); ++i) {
        Vector p0 = (i < result.node1Positions.size()) ? result.node1Positions[i] : Vector();
        Vector p1 = (i < result.node2Positions.size()) ? result.node2Positions[i] : Vector();
        double dist = (i < result.distances.size()) ? result.distances[i] : 0.0;

        out << (i + 1) << ","
            << std::fixed << std::setprecision(2) << p0.x << "," << p0.y << ","
            << p1.x << "," << p1.y << ","
            << std::setprecision(2) << dist << ","
            << std::setprecision(6) << result.throughputs[i] << ","
            << (i < result.rxPacketsPerSec.size() ? result.rxPacketsPerSec[i] : 0) << ","
            << (i < result.lostPacketsPerSec.size() ? result.lostPacketsPerSec[i] : 0)
            << "\n";
    }
    out.close();

    std::cout << "[Saved] " << fname.str() << "\n";
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

    // Station manager (rate control)
    wifi.SetRemoteStationManager(("ns3::" + wifiManager + "WifiManager").c_str());

    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    // Set up mobility (your original approach: both ConstantVelocity; node 0 stays at v=0)
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0)); // Node 0
    positionAlloc->Add(Vector(5.0, 0.0, 0.0)); // Node 1 start at x=5 m
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(nodes);

    // Node 1 moves away at 1 m/s
    Ptr<ConstantVelocityMobilityModel> mobileNode =
        nodes.Get(1)->GetObject<ConstantVelocityMobilityModel>();
    mobileNode->SetVelocity(Vector(1.0, 0.0, 0.0)); // 1 m/s along +x

    // Node 0 velocity defaults to (0,0,0) => effectively static.

    // Internet stack
    InternetStackHelper internet;
    internet.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    // Applications
    uint16_t port = 9;

    // UDP server on node 0
    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer serverApp = sink.Install(nodes.Get(0));
    serverApp.Start(Seconds(0.0));
    serverApp.Stop(Seconds(SIM_DURATION));

    // UDP client on node 1
    OnOffHelper client("ns3::UdpSocketFactory",
                       InetSocketAddress(interfaces.GetAddress(0), port));
    client.SetConstantRate(DataRate("100Mbps"), 1024);
    ApplicationContainer clientApp = client.Install(nodes.Get(1));
    clientApp.Start(Seconds(1.0));
    clientApp.Stop(Seconds(SIM_DURATION - 1));

    // FlowMonitor
    FlowMonitorHelper flowMonitor;
    Ptr<FlowMonitor> monitor = flowMonitor.InstallAll();

    // Prepare result buffers
    result.throughputs.assign(SIM_DURATION, 0.0);
    result.rxPacketsPerSec.assign(SIM_DURATION, 0);
    result.lostPacketsPerSec.assign(SIM_DURATION, 0);

    // Keep a handle to the sink to compute per-second throughput
    Ptr<PacketSink> sinkApp = DynamicCast<PacketSink>(serverApp.Get(0));
     uint64_t lastRxBytes = 0;

    // Helper to sample FlowMonitor cumulative stats and return deltas since last call
     uint64_t prevRxPkts = 0;
     uint64_t prevLostPkts = 0;

    auto sampleDrops = [&]() -> std::pair<uint64_t,uint64_t> {
        monitor->CheckForLostPackets(); // update internal stats
        uint64_t rxNow = 0, lostNow = 0;

        FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
        for (const auto &kv : stats) {
            const auto &s = kv.second;
            rxNow   += s.rxPackets;
            lostNow += s.lostPackets;
        }
        uint64_t dRx   = rxNow   - prevRxPkts;
        uint64_t dLost = lostNow - prevLostPkts;
        prevRxPkts = rxNow;
        prevLostPkts = lostNow;
        return {dRx, dLost};
    };

    // Track positions & distances every second
    for (double time = 0.0; time <= SIM_DURATION; time += 1.0) {
        Simulator::Schedule(Seconds(time), &TrackPositionsAndDistance,
                            nodes.Get(0), nodes.Get(1), std::ref(result), time);
    }

    // Sample throughput / drops once per second (for windows [t-1, t))
    for (int t = 1; t <= SIM_DURATION; ++t) {
        Simulator::Schedule(Seconds(t), [&, t]() {
            // Throughput (sink delta)
            uint64_t nowBytes = sinkApp ? sinkApp->GetTotalRx() : lastRxBytes;
            uint64_t deltaBytes = nowBytes - lastRxBytes;
            lastRxBytes = nowBytes;
            result.throughputs[t - 1] = (deltaBytes * 8.0) / 1e6; // Mbps

            // Drops (FlowMonitor delta)
            auto [dRx, dLost] = sampleDrops();
            result.rxPacketsPerSec[t - 1]   = static_cast<uint32_t>(dRx);
            result.lostPacketsPerSec[t - 1] = static_cast<uint32_t>(dLost);
        });
    }

    // Run simulation
    Simulator::Stop(Seconds(SIM_DURATION));
    Simulator::Run();

    Simulator::Destroy();

    return result;
}

int main(int argc, char *argv[]) {
    std::vector<std::string> wifiManagers = {"MinstrelHt", "ThompsonSampling", "Ideal"};
    std::vector<SimulationResult> results;

    for (uint32_t i = 0; i < wifiManagers.size(); ++i) {
        SimulationResult result = RunSimulation(wifiManagers[i], i + 1);
        results.push_back(result);
        PrintResultsToTerminal(result);
        SaveResultsToFile(result);
    }

    // Summary comparison (based on per-second Mbps)
    std::cout << "\n=== Summary Comparison ===\n";
    std::cout << std::setw(15) << "WifiManager"
              << std::setw(15) << "Avg Throughput"
              << std::setw(15) << "Max Throughput"
              << std::setw(15) << "Min Throughput\n";
    std::cout << std::string(69, '-') << "\n";

    for (const auto& result : results) {
        double sum = 0.0, maxv = 0.0;
        double minv = std::numeric_limits<double>::max();
        for (double tp : result.throughputs) {
            sum += tp;
            if (tp > maxv) maxv = tp;
            if (tp < minv) minv = tp;
        }
        double avg = sum / result.throughputs.size();
        if (minv == std::numeric_limits<double>::max()) minv = 0.0;

        std::cout << std::setw(15) << result.wifiManager
                  << std::setw(15) << std::fixed << std::setprecision(3) << avg
                  << std::setw(15) << maxv
                  << std::setw(15) << minv << "\n";
    }

    return 0;
}
