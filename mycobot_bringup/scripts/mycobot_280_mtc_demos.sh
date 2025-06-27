#!/bin/bash
# Single script to launch the mycobot with Gazebo, RViz, MoveIt 2, and the MoveIt Task Constructor demos

# Define valid exe options
valid_exe_options=("alternative_path_costs" "cartesian" "fallbacks_move_to" "ik_clearance_cost" "modular")
exe_option=${1:-"alternative_path_costs"}  # Default to alternative_path_costs if no argument provided

# Check if the provided/default argument is valid
if [[ ! " ${valid_exe_options[@]} " =~ " $exe_option " ]]; then
    echo "Invalid exe option. Available options are:"
    printf '%s\n' "${valid_exe_options[@]}"
    exit 1
fi

cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    pkill -9 -f "ros2|gazebo|gz|nav2|amcl|bt_navigator|nav_to_pose|rviz2|assisted_teleop|cmd_vel_relay|robot_state_publisher|joint_state_publisher|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator"
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

echo "Launching Gazebo simulation..."
ros2 launch mycobot_gazebo mycobot.gazebo.launch.py \
    load_controllers:=true \
    world_file:=pick_and_place_demo.world \
    use_camera:=true \
    use_rviz:=false \
    use_robot_state_pub:=true \
    use_sim_time:=true \
    x:=0.0 \
    y:=0.0 \
    z:=0.03 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0 &

sleep 15
echo "Launching the move group interface..."
ros2 launch mycobot_moveit_config move_group.launch.py \
    rviz_config_file:=mtc_demos.rviz \
    rviz_config_package:=mycobot_mtc_demos &

echo "Adjusting camera position..."
gz service -s /gui/move_to/pose \
    --reqtype gz.msgs.GUICamera \
    --reptype gz.msgs.Boolean \
    --timeout 2000 \
    --req "pose: {position: {x: 1.36, y: -0.58, z: 0.95} orientation: {x: -0.26, y: 0.1, z: 0.89, w: 0.35}}" &

sleep 10
echo "Launching the MoveIt Task Constructor demo..."
ros2 launch mycobot_mtc_demos mtc_demos.launch.py \
    use_sim_time:=true \
    exe:=$exe_option

# Keep the script running until Ctrl+C
wait
