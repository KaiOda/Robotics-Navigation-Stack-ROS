# Robotics Navigation Stack (ROS, Gazebo, RViz)

**Last updated:** 2025-10-28  

This project implements a complete **autonomous navigation pipeline** for a differential-drive robot using **ROS Noetic**, integrating **teleoperation**, **PID motion control**, **RRT path planning**, **SLAM mapping**, and **learning-based control**.

---

## Project Sections

| Module | Description |
|---------|-------------|
| **Lab 1–2 · Teleop & Goal Control** | Custom teleop node, figure-8 motion (“swim”), and goal-seeking behavior in *turtlesim*. |
| **Lab 3 · PID Motion Control** | PID controller for a differential-drive robot in Gazebo, supporting multiple operating modes. |
| **Lab 4 · RRT Path Planning** | Sampling-based motion planner over occupancy grids (`.yaml`/`.pgm`). |
| **Lab 5 · Integrated Navigation** | Full navigation stack combining RRT planner, PID controller, and RViz odometry plotting. |
| **Lab 6 · SLAM Mapping** | Autonomous mapping using *map_server*; localization and alignment through RViz. |
| **Lab 7 · Neural Network Learning** | Model-predictive control (MPC) data distilled into neural network controllers using Keras/TensorFlow. |
| **Lab 8 · Imitation Learning** | Data collection from expert trajectories and learned control via trained ROS nodes. |

---

## Technologies

- **ROS Noetic**, **Gazebo**, **RViz**
- **Python (rospy, numpy, cv2)** · **TensorFlow/Keras**
- PID control · RRT path planning · Hector SLAM · map_server
- ROS bag capture · RViz trajectory visualization

---

## ⚙️ Repository Layout

```text
src/
  teleop_control/               # Lab 1–2
  motion_planning_pid/          # Lab 3
  rrt_path_planning/            # Lab 4
  integrated_navigation_stack/  # Lab 5
  slam_mapping/                 # Lab 6
  nn_learning/                  # Lab 7
  imitation_learning/           # Lab 8
docs/
  images/                       # Screenshots & figures
  reports/                      # Optional PDF summaries
```

---

## Quick Start

```bash
# Build workspace
catkin_make
source devel/setup.bash

# Launch Gazebo world
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# Example: run full integrated navigation (Lab 5)
rosrun integrated_navigation_stack Motion_Planner.py
rosrun integrated_navigation_stack PID_Controller.py
rosrun integrated_navigation_stack RRT_node.py
```

---

## Example Figure


 `fig_turtlesim_figure8.png`
*RViz visualization showing RRT global path (green) over the SLAM-generated occupancy map. Blue traces indicate LIDAR scan data; red marker denotes the navigation goal.*

---

## Notes

- Maps stored at `src/*/maps/my_map.(pgm|yaml)`  
- RViz configuration: `src/integrated_navigation_stack/rviz_config/odom_plotter.rviz`  
- Large binaries (`.bag`, `.h5`, etc.) are ignored by `.gitignore`

---

## License
MIT — modify as appropriate

---

**Author:** Kai Oda · UC Irvine · B.S. Computer Engineering (Dec 2025)  
**GitHub:** [github.com/KaiOda](https://github.com/KaiOda)
