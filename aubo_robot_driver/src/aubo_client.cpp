#include "aubo_client.h"

namespace aubo_driver {
AuboClient::AuboClient()
{
}

bool AuboClient::init(const std::string &robot_ip, RpcClientPtr cli)
{
    cli->setRequestTimeout(1000);
    // 接口调用: 连接到 RPC 服务
    cli->connect(robot_ip, 30004);
    // 接口调用: 登录
    cli->login("aubo", "123456");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    AuboStartup(cli);

    return true;
}

void AuboClient::AuboPoweroff(RpcClientPtr cli)
{
    // 接口调用: 获取机器人的名字
    auto robot_name = cli->getRobotNames().front();
    // 接口调用: 机械臂断电
    if (cli->getRobotInterface(robot_name)->getRobotManage()->poweroff() == 0) {
        ROS_INFO_STREAM("断电成功");
    }
}

void AuboClient::AuboStartup(RpcClientPtr cli)
{
    // 接口调用: 获取机器人的名字
    auto robot_name = cli->getRobotNames().front();
    // 当机器人不处于运行模式时，执行程序
    if (RobotModeType::Running != cli->getRobotInterface(robot_name)
                                      ->getRobotState()
                                      ->getRobotModeType()) {
        // 接口调用: 发起机器人上电请求
        cli->getRobotInterface(robot_name)->getRobotManage()->poweron();
        // 循环直至机械臂上电成功
        while (1) {
            if (RobotModeType::Idle == cli->getRobotInterface(robot_name)
                                           ->getRobotState()
                                           ->getRobotModeType()) {
                ROS_INFO("上电成功:%d\n", cli->getRobotInterface(robot_name)
                                              ->getRobotState()
                                              ->getRobotModeType());
                break;
            }
            ROS_INFO("正在上电:%d\n", cli->getRobotInterface(robot_name)
                                          ->getRobotState()
                                          ->getRobotModeType());

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // 接口调用: 发起机器人启动请求
        cli->getRobotInterface(robot_name)->getRobotManage()->startup();
        // 循环直至机械臂松刹车成功
        while (1) {
            if (RobotModeType::Running == cli->getRobotInterface(robot_name)
                                              ->getRobotState()
                                              ->getRobotModeType()) {
                ROS_INFO("松刹车成功:%d\n", cli->getRobotInterface(robot_name)
                                                ->getRobotState()
                                                ->getRobotModeType());
                break;
            }
            ROS_INFO("正在松刹车:%d\n", cli->getRobotInterface(robot_name)
                                            ->getRobotState()
                                            ->getRobotModeType());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}
} // namespace aubo_driver