#include "ns3/core-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("FirstScriptExample");

int main(int argc, char *argv[])
{
    LogComponentEnable("FirstScriptExample", LOG_LEVEL_INFO);

    NS_LOG_INFO("Hello NS-3!");

    return 0;
}