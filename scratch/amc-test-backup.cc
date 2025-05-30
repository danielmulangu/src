// Standalone NS-3 5G-LENA demo to log AMC adaptation: CQI -> MCS -> TB-size
#include "ns3/core-module.h"
#include "ns3/nr-amc.h"
#include <iostream>
#include <vector>

using namespace ns3;

int
main (int argc, char *argv[])
{
    // Enable logging to see internal AMC decisions
    LogComponentEnable ("NrAmc", LOG_LEVEL_INFO);

    // Create and configure the AMC model
    Ptr<NrAmc> amc = CreateObject<NrAmc> ();
    // Select error-based AMC algorithm
    amc->SetAmcModel (NrAmc::ErrorModel);
    // Operate in downlink mode
    amc->SetDlMode ();
    // Default DM-RS symbols per RB = 1
    amc->SetNumRefScPerRb (1);

    const uint32_t numRb = 50;  // Resource Blocks allocated

    // Define a range of CQI values to simulate channel quality
    std::vector<uint8_t> cqiValues = {1, 3, 5, 7, 9, 11, 13, 15};

    // Print header
    std::cout << "CQI\tMCS\tTBSize(bits)\n";

    // Loop over CQI values and show adaptation mapping
    for (uint8_t cqi : cqiValues)
    {
        // Map CQI -> MCS index
        uint8_t mcs = amc->GetMcsFromCqi (cqi);
        // Calculate Transport Block size (layers=1)
        uint32_t tbs = amc->CalculateTbSize (mcs, 1, numRb);
        // Print results
        std::cout << static_cast<uint32_t> (cqi)
                  << "\t" << static_cast<uint32_t> (mcs)
                  << "\t" << tbs << "\n";
    }

    return 0;
}
