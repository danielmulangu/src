Read Me for the backup of NS-3
Note: This repo will keep all my NS-3 files that i have been using in my training of networking and particularly ML Networks.

//

The ns3_shared_memory.py is a simple progrsam that reads data from a memory that was created in a C++ program in NS-3. Its sole purpose is to display the dynamic of the shared memory mechanism that is discussed in NS3-AI.

The program starts by reading the data that was primarly written to that shared memory by ns-3 and then display the content. Given the ID of that shared memory. It then, update it to a newer value before displaying it and then let the C program reads the updated value
