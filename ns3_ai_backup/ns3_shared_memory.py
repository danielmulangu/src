#
# Created by Daniel Mulangu on 3/19/25.
#

import py_interface
from ctypes import *
import time

# Define parameters for the shared memory
mempool_key = 1234  # Arbitrary integer larger than 1000
mem_size = 4096  # Memory pool size
memblock_key = 2  # Shared Memory ID

# Initialize shared memory
py_interface.Init(mempool_key, mem_size)

# Create a shared memory variable
my_var = py_interface.NS3Var(memblock_key, c_int)
time.sleep(5)
new_value = my_var.Get()
print(f"Python: NS3 wrote initially {new_value} from shared memory.")
# Write an initial value
my_var.Set(c_int(40))
print("Python: Wrote 40 to shared memory.")

# Simulate waiting for NS-3 to modify the value

time.sleep(10)  # Wait for NS-3 to change the value

# Read the updated value from shared memory
new_value1 = my_var.Get()
print(f"Python: NS3 updated value to {new_value1} from shared memory.")

# Cleanup shared memory
py_interface.FreeMemory()
