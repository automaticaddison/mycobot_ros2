/**
 * @file arm_gripper_loop_controller.cpp
 * @brief Control robot arm and gripper to perform repetitive movements between positions.
 *
 * This program creates a ROS 2 node that moves a robot arm between target and home positions,
 * coordinating with gripper actions (open/close) at each position. The movement happens
 * in a continuous loop.
 *
 * Action Clients:
 *     /arm_controller/follow_joint_trajectory (control_msgs/FollowJointTrajectory):
 *         Commands for controlling arm joint positions
 *     /gripper_action_controller/gripper_cmd (control_msgs/GripperCommand):
 *         Commands for opening and closing the gripper
 *
 * @author Addison Sears-Collins
 * @date November 16, 2024
 */

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"

using namespace std::chrono_literals;

/**
 * @brief A ROS 2 node for controlling robot arm movements and gripper actions.
 *
 * This class creates a simple control loop that:
 * 1. Moves the arm to a target position
 * 2. Closes the gripper
 * 3. Returns the arm to home position
 * 4. Opens the gripper
 */
class ArmGripperLoopController : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GripperCommand = control_msgs::action::GripperCommand;

    /**
     * @brief Constructor for the ArmGripperLoopController class.
     *
     * Initializes the node and sets up action clients for arm and gripper control.
     * Sets up two action clients:
     * - One for controlling arm movements
     * - One for controlling gripper open/close actions
     * Also defines the positions for movement and starts a timer for the control loop.
     */
    ArmGripperLoopController()
        : Node("arm_gripper_loop_controller")
    {
        // Set up action clients
        arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/arm_controller/follow_joint_trajectory"
        );

        gripper_client_ = rclcpp_action::create_client<GripperCommand>(
            this,
            "/gripper_action_controller/gripper_cmd"
        );

        // Wait for action servers
        RCLCPP_INFO(this->get_logger(), "Waiting for action servers...");
        if (!arm_client_->wait_for_action_server(20s)) {
            RCLCPP_ERROR(this->get_logger(), "Arm action server not available");
            return;
        }
        if (!gripper_client_->wait_for_action_server(20s)) {
            RCLCPP_ERROR(this->get_logger(), "Gripper action server not available");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Action servers connected!");

        // Initialize joint names
        joint_names_ = {
            "link1_to_link2", "link2_to_link3", "link3_to_link4",
            "link4_to_link5", "link5_to_link6", "link6_to_link6_flange"
        };

        // Define positions
        target_pos_ = {1.345, -1.23, 0.264, -0.296, 0.389, -1.5};
        home_pos_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // Create timer for control loop
        timer_ = this->create_wall_timer(
            100ms,  // 0.1 seconds
            std::bind(&ArmGripperLoopController::controlLoopCallback, this)
        );
    }

private:
    /**
     * @brief Send a command to move the robot arm to specified joint positions.
     *
     * Creates and sends a trajectory goal message to move the arm to the desired position.
     * The movement is configured to take 2 seconds to complete.
     *
     * @param positions Vector of 6 joint angles in radians
     */
    void sendArmCommand(const std::vector<double>& positions)
    {
        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.time_from_start.sec = 2;  // 2 seconds for movement
        goal_msg.trajectory.points.push_back(point);

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        arm_client_->async_send_goal(goal_msg, send_goal_options);
    }

    /**
     * @brief Send a command to the gripper to open or close.
     *
     * Creates and sends a gripper command with the specified position and a fixed max effort.
     *
     * @param position Position value for gripper (0.0 for open, -0.7 for closed)
     */
    void sendGripperCommand(double position)
    {
        auto goal_msg = GripperCommand::Goal();
        goal_msg.command.position = position;
        goal_msg.command.max_effort = 5.0;

        auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
        gripper_client_->async_send_goal(goal_msg, send_goal_options);
    }

    /**
     * @brief Execute one cycle of the control loop.
     *
     * This callback performs the following sequence:
     * 1. Move arm to target position (2.5s movement)
     * 2. Pause at target (1s)
     * 3. Close gripper (0.5s)
     * 4. Move arm to home position (2.5s movement)
     * 5. Pause at home (1s)
     * 6. Open gripper (0.5s)
     * 7. Pause before next cycle (1s)
     */
    void controlLoopCallback()
    {
        // Move to target position
        RCLCPP_INFO(this->get_logger(), "Moving to target position");
        sendArmCommand(target_pos_);
        std::this_thread::sleep_for(2500ms);  // 2.5s wait

        // Pause at target
        RCLCPP_INFO(this->get_logger(), "Reached target position - Pausing");
        std::this_thread::sleep_for(1000ms);  // 1s pause

        // Close gripper
        RCLCPP_INFO(this->get_logger(), "Closing gripper");
        sendGripperCommand(-0.7);
        std::this_thread::sleep_for(500ms);  // 0.5s wait

        // Move to home position
        RCLCPP_INFO(this->get_logger(), "Moving to home position");
        sendArmCommand(home_pos_);
        std::this_thread::sleep_for(2500ms);  // 2.5s wait

        // Pause at home
        RCLCPP_INFO(this->get_logger(), "Reached home position - Pausing");
        std::this_thread::sleep_for(1000ms);  // 1s pause

        // Open gripper
        RCLCPP_INFO(this->get_logger(), "Opening gripper");
        sendGripperCommand(0.0);
        std::this_thread::sleep_for(500ms);  // 0.5s wait

        // Final pause
        std::this_thread::sleep_for(1000ms);  // 1s pause
    }

    // Action clients
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Joint names and positions
    std::vector<std::string> joint_names_;
    std::vector<double> target_pos_;
    std::vector<double> home_pos_;
};

/**
 * @brief Main function to initialize and run the arm gripper control node.
 *
 * Initializes ROS 2, creates the controller node, and spins it until shutdown.
 * Includes exception handling for robustness.
 *
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return 0 on successful execution, non-zero on error
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmGripperLoopController>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
