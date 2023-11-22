#ifndef AUBO_HARDWARE_INTERFACE_H
#define AUBO_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>
#include <pass_through_controllers/trajectory_interface.h>
#include <algorithm>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryFeedback.h>

#include <aubo_msgs/IOStates.h>
#include <aubo_msgs/ToolDataMsg.h>
#include <aubo_msgs/SetIO.h>
#include <aubo_msgs/SetSpeedSliderFraction.h>
#include <aubo_msgs/SetPayload.h>

#include <cartesian_interface/cartesian_command_interface.h>
#include <cartesian_interface/cartesian_state_handle.h>

#include <speed_scaling_interface/speed_scaling_interface.h>
#include <scaled_joint_trajectory_controller/scaled_joint_command_interface.h>

#include <aubo_dashboard_msgs/RobotMode.h>
#include <aubo_dashboard_msgs/SafetyMode.h>

#include <industrial_robot_status_interface/industrial_robot_status_interface.h>
#include <kdl/frames.hpp>
#include <mutex>

#include <atomic>

#include <aubo/robot/robot_state.h>
#include "aubo_sdk/rtde.h"
#include "aubo_sdk/rpc.h"
#include "serviceinterface.h"
#include "thread"

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;

namespace aubo_driver {
class AuboHardwareInterface : public hardware_interface::RobotHW
{
public:
    AuboHardwareInterface();
    AuboHardwareInterface(ros::NodeHandle &nh);
    virtual ~AuboHardwareInterface() = default;
    /*!
     * \brief Handles the setup functionality for the ROS interface. This
     * includes parsing ROS parameters, creating interfaces, starting the main
     * driver and advertising ROS services.
     *
     * \param root_nh Root level ROS node handle
     * \param robot_hw_nh ROS node handle for the robot namespace
     *
     * \returns True, if the setup was performed successfully
     */
    virtual bool init(ros::NodeHandle &root_nh,
                      ros::NodeHandle &robot_hw_nh) override;
    /*!
     * \brief Read method of the control loop. Reads a RTDE package from the
     * robot and handles and publishes the information as needed.
     *
     * \param time Current time
     * \param period Duration of current control loop iteration
     */
    virtual void read(const ros::Time &time,
                      const ros::Duration &period) override;
    /*!
     * \brief Write method of the control loop. Writes target joint positions to
     * the robot to be read by its URCaps program.
     *
     * \param time Current time
     * \param period Duration of current control loop iteration
     */
    virtual void write(const ros::Time &time,
                       const ros::Duration &period) override;
    /*!
     * \brief Preparation to start and stop loaded controllers.
     *
     * \param start_list List of controllers to start
     * \param stop_list List of controllers to stop
     *
     * \returns True, if the controllers can be switched
     */
    virtual bool prepareSwitch(
        const std::list<hardware_interface::ControllerInfo> &start_list,
        const std::list<hardware_interface::ControllerInfo> &stop_list)
        override;
    /*!
     * \brief Starts and stops controllers.
     *
     * \param start_list List of controllers to start
     * \param stop_list List of controllers to stop
     */
    virtual void doSwitch(
        const std::list<hardware_interface::ControllerInfo> &start_list,
        const std::list<hardware_interface::ControllerInfo> &stop_list)
        override;

    void readActualQ();
    void configSubscribe(RtdeClientPtr cli);
    void setInput(RtdeClientPtr cli);

    bool isRobotProgramRunning() const;

    bool checkControllerClaims(const std::set<std::string> &claimed_resources);

    template <typename T>
    void printVec(std::vector<T> param, std::string name);

    bool shouldResetControllers();

    int Servoj(const std::array<double, 6> joint_position_command);

    int startServoMode();

    int stopServoMode();

    bool isServoModeStart();

protected:
    ros::NodeHandle nh_;    //
    ros::Timer timer_;      //定期更新的定时器
    double control_period_; //控制周期
    ros::Duration elapsed_time_;
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

private:
    std::shared_ptr<RpcClient> rpc_client_{ nullptr };
    std::shared_ptr<RtdeClient> rtde_client_{ nullptr };
    std::vector<std::string> joint_names_;
    std::mutex rtde_mtx_;
    std::string robot_ip_;

    hardware_interface::JointStateInterface js_interface_;
    hardware_interface::PositionJointInterface pj_interface_;
    scaled_controllers::ScaledPositionJointInterface spj_interface_;
    std::array<double, 6> joint_position_command_;
    double speed_scaling_combined_;
    bool controllers_initialized_;
    bool servo_mode_start_{ false };

    std::atomic<bool> robot_program_running_;
    std::atomic<bool> controller_reset_necessary_{ false };
    //    uint32_t runtime_state_;
    std::atomic<bool> position_controller_running_;
    std::atomic<bool> velocity_controller_running_;
    std::atomic<bool> joint_forward_controller_running_;
    std::atomic<bool> cartesian_forward_controller_running_;
    std::atomic<bool> twist_controller_running_;
    std::atomic<bool> pose_controller_running_;

    // topic1
    int line_{ -1 };
    std::vector<double> actual_q_{ std::vector<double>(6, 0) };
    std::array<double, 6> actual_q_copy_;
    std::vector<double> actual_qd_{ std::vector<double>(6, 0) };
    std::vector<double> target_q_{ std::vector<double>(6, 0) };
    std::vector<double> target_qd_{ std::vector<double>(6, 0) };

    std::vector<double> actual_current_{ std::vector<double>(6, 0.1) };
    std::vector<double> actual_current_e{ std::vector<double>(6, 0) };
    std::vector<double> actual_TCP_pose_{ std::vector<double>(6, 0.) };
    std::vector<double> actual_TCP_speed_{ std::vector<double>(6, 0.) };
    std::vector<double> actual_TCP_force_{ std::vector<double>(6, 0.) };
    std::vector<double> target_TCP_pose_{ std::vector<double>(6, 0.) };
    std::vector<double> target_TCP_speed_{ std::vector<double>(6, 0.) };

    RobotModeType robot_mode_ = RobotModeType::NoController;
    SafetyModeType safety_mode_ = SafetyModeType::Normal;
    RuntimeState runtime_state_ = RuntimeState::Stopped;
};
} // namespace aubo_driver
#endif // AUBO_HARDWARE_INTERFACE_H
