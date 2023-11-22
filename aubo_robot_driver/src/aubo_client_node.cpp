#include "aubo_client.h"
#include <csignal>

std::unique_ptr<aubo_driver::AuboClient> aubo_client;
// aubo_driver::AuboClient aubo_client;
auto rpc_client_ = std::make_shared<RpcClient>();
void signalHandler(int signum)
{
    //机器人断电
    aubo_client->AuboPoweroff(rpc_client_);
    ROS_INFO_STREAM("Interrupt signal (" << signum << ") received.\n");
    exit(signum);
}

int main(int argc, char *argv[])
{
    //防止ROS_INFO中文打印乱码
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "aubo_client");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle nh("~");
    // std::string robot_ip = nh.param<std::string>("robot_ip", "172.16.3.83");
    std::string robot_ip = argv[1];
    signal(SIGINT, signalHandler);
    if (!aubo_client->init(robot_ip, rpc_client_)) {
        ROS_ERROR_STREAM("Could not correctly initialize robot. Exiting");
        exit(1);
    }

    while (ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    aubo_client->AuboPoweroff(rpc_client_);
    spinner.stop();
    return 0;
}