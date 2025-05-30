#include "train-var.h"
#include <iostream>
#include <unistd.h>

using namespace ns3;

int main () {
    // Create a TrainVar instance for the same shared memory (id 1)
    TrainVar<int> myVar(2);

    // Read the value from shared memory.
    int readValue = myVar.Get();
    std::cout << "Reader: Read " << readValue << " from shared memory." << std::endl;

    // Reset the write tag to allow an update from the writer.
    myVar.m_data->tagWt = SETABLE;  // Manually reset tagWt

    // Multiply the read value.
    int updatedValue = readValue * 2;

    // Write the updated value back into shared memory.
    myVar.Set(updatedValue);
    std::cout << "Reader: Updated shared memory with value " << updatedValue << "." << std::endl;

    sleep(20);
    return 0;
}
