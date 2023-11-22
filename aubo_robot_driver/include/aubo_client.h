#include "aubo_sdk/rpc.h"
#include <ros/ros.h>

using namespace arcs::aubo_sdk;

namespace aubo_driver {
class AuboClient
{
public:
    AuboClient();
    ~AuboClient() = default;
    bool init(const std::string &robot_ip, RpcClientPtr cli);
    void AuboStartup(RpcClientPtr cli);
    void AuboPoweroff(RpcClientPtr cli);
};

} // namespace aubo_driver
