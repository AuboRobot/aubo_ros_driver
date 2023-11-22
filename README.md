# aubo_ros_driver

遨博机器人ROS驱动,在kinetic\melotic\noetic上通过编译、运行测试

# 安装所需工具(修改对应ros版本)
```bash
安装工业机器人控制接口：
sudo apt install ros-kinetic-industrial-robot-status-interface
安装moveit：
sudo apt install ros-kinetic-moveit
安装逆向运动学求解器：
sudo apt install ros-kinetic-trac-ik-kinematics-plugin
```
# 在 gazebo 中仿真 aubo 机器人（以 aubo_i5 为例）
```bash
roslaunch aubo_gazebo aubo_bringup.launch robot_model:=aubo_i5
roslaunch aubo_moveit_config moveit_planning_execution.launch sim:=true robot_model:=aubo_i5
roslaunch aubo_moveit_config moveit_rviz.launch
```
# 驱动真实机械臂 aubo_i5 (修改机器人对应 robot_ip )
```bash 
roslaunch aubo_robot_driver aubo_bringup.launch robot_model:=aubo_i5 robot_ip:=192.168.127.128  debug:=false aubo_hardware_interface_node_required:=false
roslaunch aubo_moveit_config moveit_planning_execution.launch robot_model:=aubo_i5
roslaunch aubo_moveit_config moveit_rviz.launch
```
# 驱动真实机械臂单点轨迹运动demo (修改机器人对应 robot_ip robot_model)
```bash 
roslaunch aubo_robot_driver aubo_bringup.launch robot_model:=aubo_i5 robot_ip:=192.168.127.128  debug:=false aubo_hardware_interface_node_required:=false
roslaunch aubo_planning aubo_planning_demo.launch robot_model:=aubo_i5

```
