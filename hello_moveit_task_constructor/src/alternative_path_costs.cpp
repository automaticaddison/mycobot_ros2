#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene/planning_scene.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/fixed_state.h>

#include <moveit/task_constructor/cost_terms.h>

#include <iostream> // Added for debug output

using namespace moveit::task_constructor;

/* FixedState - Connect - FixedState */
int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    std::cout << "ROS 2 initialized for alternative_path_costs_demo" << std::endl;
    
    // Set up ROS 2 node options
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    // Create a ROS 2 node
    auto node = rclcpp::Node::make_shared("alternative_path_costs_demo", node_options);
    std::cout << "Created ROS 2 node: alternative_path_costs_demo" << std::endl;
    
    // Start a separate thread to handle ROS 2 callbacks
    std::thread spinning_thread([node] { rclcpp::spin(node); });

    // Create a Task Constructor task
    Task t;
    t.stages()->setName("alternative path costs");
    std::cout << "Created Task Constructor task: alternative path costs" << std::endl;
    
    // Load the robot model
    t.loadRobotModel(node);
    std::cout << "Loaded robot model: " << t.getRobotModel()->getName() << std::endl;

    // Ensure the correct robot model is loaded
    assert(t.getRobotModel()->getName() == "mycobot_280");

    // Create a planning scene
    auto scene{ std::make_shared<planning_scene::PlanningScene>(t.getRobotModel()) };
    std::cout << "Created planning scene" << std::endl;
    
    // Get the current robot state and set it to default values
    auto& robot_state{ scene->getCurrentStateNonConst() };
    robot_state.setToDefaultValues();
    robot_state.setToDefaultValues(robot_state.getJointModelGroup("arm"), "home");
    std::cout << "Set robot state to default values and 'home' position" << std::endl;

    // Create and add the initial state to the task
    auto initial{ std::make_unique<stages::FixedState>("start") };
    initial->setState(scene);
    t.add(std::move(initial));
    std::cout << "Added initial state to the task" << std::endl;

    // Create a pipeline planner
    // The "pipeline" part means it can chain together multiple planning attempts or strategies. 
    // If one method fails, it can try another
    auto pipeline{ std::make_shared<solvers::PipelinePlanner>(node) };
    std::cout << "Created pipeline planner" << std::endl;

    // Create an Alternatives container for different path planning strategies
    // This container allows you to define multiple ways to solve a problem. 
    // In this case, it's used to create different strategies for path planning. 
    // The planner will try all these strategies and choose the best one based on their respective cost terms.
    auto alternatives{ std::make_unique<Alternatives>("connect") };
    std::cout << "Created Alternatives container for path planning strategies" << std::endl;
    
    // Add different path planning strategies (Connect stages) to the Alternatives container
    // Each strategy uses a different cost term to evaluate the path

    // Strategy 1: Minimize path length    
    {
        auto connect{ std::make_unique<stages::Connect>(
             "path length", stages::Connect::GroupPlannerVector{ { "arm_with_gripper", pipeline } }) };
        connect->setCostTerm(std::make_unique<cost::PathLength>());
        alternatives->add(std::move(connect));
        std::cout << "Added 'path length' strategy" << std::endl;
    }

    // Strategy 2: Minimize trajectory duration
    {
        auto connect{ std::make_unique<stages::Connect>(
             "trajectory duration", stages::Connect::GroupPlannerVector{ { "arm_with_gripper", pipeline } }) };
        connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
        alternatives->add(std::move(connect));
        std::cout << "Added 'trajectory duration' strategy" << std::endl;
    }
    
    // Strategy 3: Minimize end-effector motion
    {
        auto connect{ std::make_unique<stages::Connect>(
             "eef motion", stages::Connect::GroupPlannerVector{ { "arm_with_gripper", pipeline } }) };
        connect->setCostTerm(std::make_unique<cost::LinkMotion>("link6_flange"));
        alternatives->add(std::move(connect));
        std::cout << "Added 'end-effector motion' strategy" << std::endl;
    }
    
    // Strategy 4: Minimize elbow motion
    {
        auto connect{ std::make_unique<stages::Connect>(
             "elbow motion", stages::Connect::GroupPlannerVector{ { "arm_with_gripper", pipeline } }) };
        connect->setCostTerm(std::make_unique<cost::LinkMotion>("link3"));
        alternatives->add(std::move(connect));
        std::cout << "Added 'elbow motion' strategy" << std::endl;
    }

    // Add the Alternatives container to the task
    t.add(std::move(alternatives));
    std::cout << "Added all strategies to the task" << std::endl;

    // Create the goal scene as a diff from the current scene
    auto goal_scene{ scene->diff() };
    goal_scene->getCurrentStateNonConst().setToDefaultValues(robot_state.getJointModelGroup("arm_with_gripper"), "ready");
    std::cout << "Created goal scene with 'ready' position" << std::endl;
    
    // Create and add the goal state to the task
    auto goal = std::make_unique<stages::FixedState>("goal");
    goal->setState(goal_scene);
    t.add(std::move(goal));
    std::cout << "Added goal state to the task" << std::endl;

    // Plan the task
    std::cout << "Starting task planning..." << std::endl;
    try {
        t.plan(0);
        std::cout << "Task planning completed successfully" << std::endl;
        
        // Print the results
        std::cout << "Planning results:" << std::endl;
        t.printState();
                
    } catch (const InitStageException& e) {
        std::cout << "Task planning failed: " << e << std::endl;
    }
    
    // Keep the node alive for interactive inspection in RViz
    std::cout << "Keeping node alive for RViz inspection. Press Ctrl+C to exit." << std::endl;
    spinning_thread.join();

    return 0;
}

