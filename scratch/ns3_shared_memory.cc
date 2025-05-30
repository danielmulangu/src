//
//  Created by Daniel Mulangu on 3/19/25.
#include "train-var.h"
#include <iostream>
#include <unistd.h>  // for sleep

using namespace ns3;

int main () {
    // Create a TrainVar instance for an integer in shared memory with id = 1
    TrainVar<int> myVar(2);

    // Initialize both tags to SETABLE.
    myVar.m_data->tagRd = SETABLE;
    myVar.m_data->tagWt = SETABLE;

    int value = 20;
    // Write the value into shared memory.
    myVar.Set(value);

    std::cout << "NS3: Wrote " << value << " to shared memory." << std::endl;

    //Read the new value changed by Python
    TrainVar<int> myVar2(2);
    myVar2.m_data->tagRd = READABLE;
    int readValue = myVar2.Get();
    while (readValue ==value)
    {
        readValue = myVar2.Get();
    }
    std::cout << "NS3: Python changed value to: " << readValue << " from shared memory." << std::endl;
    // Reset the write tag to allow an update from the writer.
    myVar2.m_data->tagWt = SETABLE;  // Manually reset tagWt

    //Update the new value to 45
    TrainVar<int> myVar3(2);
    myVar3.m_data->tagRd = SETABLE;
    myVar3.m_data->tagWt = SETABLE;
    int val = 45;
    myVar.Set(val);
    myVar3.m_data->tagRd = READABLE;
    std::cout << "NS3: Wrote " << val << " to shared memory." << std::endl;
    sleep(10);
    return 0;
}
