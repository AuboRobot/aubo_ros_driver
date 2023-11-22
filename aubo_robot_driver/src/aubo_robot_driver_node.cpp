#include <ros/callback_queue.h>
#include <csignal>
#include <aubo_hardware_interface.h>

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;
using RtdeRecipeMap =
    std::unordered_map<int, arcs::common_interface::RtdeRecipe>;

std::unique_ptr<aubo_driver::AuboHardwareInterface> g_aubo_hw_interface;

void signalHandler(int signum)
{
    
    std::cout << "Interrupt signal (" << signum << ") received.\n";

    // cleanup and close up stuff here
    // terminate program

    exit(signum);
}

int main(int argc, char **argv)
{
    std::string robot_ip_ = "192.168.10.135lm";
    ros::init(argc, argv, "aubo_hardware_interface");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle nh;
    ros::NodeHandle robot_hw_nh("~");

    signal(SIGINT, signalHandler);

    //    std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
    //    bool has_realtime = false;
    //    if (realtime_file.is_open()) {
    //        realtime_file >> has_realtime;
    //    }
    if (/*has_realtime*/ 1) {
        const int max_thread_priority = sched_get_priority_max(SCHED_FIFO);
        if (max_thread_priority != -1) {
            // We'll operate on the currently running thread.
            pthread_t this_thread = pthread_self();

            // struct sched_param is used to store the scheduling priority
            struct sched_param params;

            // We'll set the priority to the maximum.
            params.sched_priority = max_thread_priority;

            int ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
            if (ret != 0) {
                ROS_ERROR_STREAM("Unsuccessful in setting main thread realtime "
                                 "priority. Error code: "
                                 << ret);
            }
            // Now verify the change in thread priority
            int policy = 0;
            ret = pthread_getschedparam(this_thread, &policy, &params);
            if (ret != 0) {
                std::cout << "Couldn't retrieve real-time scheduling paramers"
                          << std::endl;
            }

            // Check the correct policy was applied
            if (policy != SCHED_FIFO) {
                ROS_ERROR("Main thread: Scheduling is NOT SCHED_FIFO!");
            } else {
                ROS_INFO("Main thread: SCHED_FIFO OK");
            }

            // Print thread scheduling priority
            ROS_INFO_STREAM("Main thread priority is "
                            << params.sched_priority);
        } else {
            ROS_ERROR("Could not get maximum thread priority for main thread");
        }
    }
    // Set up timers
    ros::Time timestamp;
    ros::Duration period;
    auto stopwatch_last = std::chrono::steady_clock::now();
    auto stopwatch_now = stopwatch_last;

    g_aubo_hw_interface.reset(
        new aubo_driver::AuboHardwareInterface(robot_hw_nh));

    if (!g_aubo_hw_interface->init(nh, robot_hw_nh)) {
        ROS_ERROR_STREAM("Could not correctly initialize robot. Exiting");
        exit(1);
    }
    ROS_DEBUG_STREAM("initialized hw interface");
    controller_manager::ControllerManager cm(g_aubo_hw_interface.get(), nh);
    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(
                       stopwatch_now - stopwatch_last)
                       .count());
    stopwatch_last = stopwatch_now;

    g_aubo_hw_interface->startServoMode();
    while (ros::ok()) {
        g_aubo_hw_interface->read(timestamp, period);
        timestamp = ros::Time::now();
        stopwatch_now = std::chrono::steady_clock::now();
        period.fromSec(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                stopwatch_now - stopwatch_last)
                .count());
        stopwatch_last = stopwatch_now;
        cm.update(timestamp, period,
                  g_aubo_hw_interface->shouldResetControllers());

        g_aubo_hw_interface->write(timestamp, period);
    }
    g_aubo_hw_interface->stopServoMode();
    spinner.stop();

    return 0;
}
