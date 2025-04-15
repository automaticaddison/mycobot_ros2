#!/bin/bash
# Single script to launch point cloud viewing

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
    z:=0.0 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0 &

sleep 15
echo "Adjusting camera position..."
gz service -s /gui/move_to/pose \
    --reqtype gz.msgs.GUICamera \
    --reptype gz.msgs.Boolean \
    --timeout 2000 \
    --req "pose: {position: {x: 1.36, y: -0.58, z: 0.95} orientation: {x: -0.26, y: 0.1, z: 0.89, w: 0.35}}" &

sleep 10

# Select one of the commands below. Comment out the others

# See the full point cloud
#ros2 launch mycobot_mtc_pick_place_demo point_cloud_viewer.launch.py file_name:=/tmp/4_convertToPCL_debug_cloud.pcd

# See the support plane that was extracted from the point cloud
#ros2 launch mycobot_mtc_pick_place_demo point_cloud_viewer.launch.py file_name:=/tmp/5_support_plane_debug_cloud.pcd

# See the objects cloud that was extracted from the point cloud
ros2 launch mycobot_mtc_pick_place_demo point_cloud_viewer.launch.py file_name:=/tmp/5_objects_cloud_debug_cloud.pcd
