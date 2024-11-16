# mycobot_system_tests

Integration and system tests for verifying myCobot robot functionality.

## Overview

This package provides system-level integration tests for the myCobot robotic arm,
focusing on verifying joint movements, gripper operations, and coordinated actions.
The tests help ensure reliable operation of the robot in production environments.

## Python Example Test
```bash
ros2 run mycobot_system_tests arm_gripper_loop_controller.py
```

## C++ Example Test
```bash
ros2 run mycobot_system_tests arm_gripper_loop_controller
```