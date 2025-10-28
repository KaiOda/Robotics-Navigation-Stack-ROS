# Robotics Navigation Stack (ROS, Gazebo, RViz)

**Last updated:** 2025-10-28

This repository implements an end-to-end autonomous navigation pipeline for a
differential-drive robot using ROS Noetic, integrating teleoperation, PID motion
control, RRT planning, SLAM, and learning-based control.

## Project Modules

| Module | Description |
|-------|-------------|
| **Lab 1-2: Teleop & Goal Control** | Custom teleop, figure-8 swim, and goal-seeking on turtlesim. |
| **Lab 3: PID Motion Control** | PID control of differential drive in Gazebo; mode-0/1 behaviors. |
| **Lab 4: RRT Path Planning** | Sampling-based planner over occupancy grids (yaml/pgm). |
| **Lab 5: Integrated Navigation** | RRT planner + PID controller + RViz odometry plotting. |
| **Lab 6: SLAM Mapping** | Autonomous mapping; map_server save + localization alignment. |
| **Lab 7: NN Learning** | MPC data -> NN controller (Keras/TensorFlow). |
| **Lab 8: Imitation Learning** | ROS bag collection, dataset build, NN control in ROS node. |

## Technologies
- ROS Noetic, Gazebo, RViz
- Python (rospy, numpy, cv2), TensorFlow/Keras
- PID control, RRT path planning, Hector SLAM, map_server
- ROS bags for data capture; RViz trajectory visualization

## Repository Layout
```
src/
  teleop_control/              # Lab1-2
  motion_planning_pid/         # Lab3
  rrt_path_planning/           # Lab4
  integrated_navigation_stack/  # Lab5
  slam_mapping/                # Lab6
  nn_learning/                 # Lab7
  imitation_learning/          # Lab8
docs/
images/                         # screenshots, figures
reports/                        # PDF summaries (optional)
```

## Quickstart
```bash
# From repo root
catkin_make
source devel/setup.bash

# Gazebo world
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# Example: integrated navigation (Lab 5)
rosrun integrated_navigation_stack Motion_Planner.py
rosrun integrated_navigation_stack PID_Controller.py
rosrun integrated_navigation_stack RRT_node.py
```

## Example Figures
See docs/images/ (place your screenshots):

- fig_turtlesim_figure8.png - turtlesim figure-8
- fig_pid_tracking.png - PID trajectory tracking
- fig_rrt_path_overlay.png - RRT path on occupancy map
- fig_rviz_vs_rrt.png - RViz odometry vs planned path
- fig_nn_trajectory.png - learned controller trajectory

## Notes
- Maps: src/*/maps/my_map.(pgm|yaml)
- RViz config: src/integrated_navigation_stack/rviz_config/odom_plotter.rviz
- Large binaries (bags, h5) are ignored by .gitignore by default.

## License
MIT (adjust as needed)

Author: Kai Oda - UC Irvine - B.S. Computer Engineering (Dec 2025)
GitHub: https://github.com/KaiOda
