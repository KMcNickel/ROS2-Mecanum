#/bin/bash

ros2 lifecycle set /agv0/inverse_kinematics configure
ros2 lifecycle set /agv0/forward_kinematics configure
ros2 lifecycle set /agv0/inverse_kinematics activate
ros2 lifecycle set /agv0/forward_kinematics activate