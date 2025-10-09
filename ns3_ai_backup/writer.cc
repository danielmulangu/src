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

    // Manually set tagRd to READABLE so that the reader can read the data.
    myVar.m_data->tagRd = READABLE;

    std::cout << "Writer program: Wrote " << value << " to shared memory." << std::endl;

    // Keep the writer running so the shared memory remains active.
    sleep(10);
    //Read the new value changed by the reader
    TrainVar<int> myVar2(2);
    myVar2.m_data->tagRd = READABLE;
    int readValue = myVar2.Get();
    std::cout << "Reader changed value to: " << readValue << " from shared memory." << std::endl;

    sleep(10);
    return 0;
}
