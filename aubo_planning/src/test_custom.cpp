#include <moveit/move_group_interface/move_group_interface.h>
#include <csignal>
#include <thread>




void signalHandler(int signum)
{
    
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    ros::shutdown();
    // cleanup and close up stuff here
    // terminate program

    exit(signum);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_custom_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    signal(SIGINT, signalHandler);
    
    moveit::planning_interface::MoveGroupInterface group("manipulator_C3");
    // 设置机器人各个关节的目标位置 : shoulder_joint  upperArm_joint foreArm_joint wrist1_joint wrist2_joint wrist3_joint   
    std::vector<double> target_joint1_ = {0.01153122,0.01162875,0.011556579999999999,-0.01163079,-0.01165532,0.01155502};
    std::vector<double> target_joint2_ = {-2.32583327,-0.01165322,-1.4925959099999997,0.01182081,-1.5852720999999876,0.01163905};
    //设置随机目标
    //group.setRandomTarget();
    // 设置机器人终端的目标位置
    //geometry_msgs::Pose target_pose1;
    //target_pose1.orientation.w = 0.726282;
    //target_pose1.orientation.x = 4.04423e-07;
    //target_pose1.orientation.y = -0.687396;
    //target_pose1.orientation.z = 4.81813e-07;

    //target_pose1.position.x = 0.03;
    //target_pose1.position.y = 0.2;
    //target_pose1.position.z = 0.5;
    //group.setPoseTarget(target_pose1);

    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

    //ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    //让机械臂按照规划的轨迹开始运动。
    //if (success)
    //    group.execute(my_plan);
    
     //设置关节速度0-1
    group.setMaxVelocityScalingFactor(0.5);  
    while(ros::ok()){
        
         group.setJointValueTarget(target_joint1_);
    
         group.move(); //plan and execute
         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
         group.setJointValueTarget(target_joint2_);
         group.move();
         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    }
    spinner.stop();
    
    return 0;
}
