#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
from rrt import rrt_pathfinder
import sys
import cv2
import os

class RRTNode:
    def __init__(self):
        rospy.init_node('rrt_node', anonymous=True)

        self.map_received = False
        self.map_img = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/start_goal', Float64MultiArray, self.start_goal_callback)

        self.trajectory_pub = rospy.Publisher('/trajectory', Float64MultiArray, queue_size=10)

        rospy.loginfo("RRT Node Initialized. Waiting for map...")

    def map_callback(self, msg):
        if self.map_received:
            return

        rospy.loginfo("Map received!")
        self.resolution = msg.info.resolution
        x_origin = msg.info.origin.position.x
        y_origin = msg.info.origin.position.y
        self.origin = [x_origin, y_origin]
        self.width = msg.info.width
        self.height = msg.info.height
        data = np.array(msg.data)

        grid_2d = data.reshape((self.height, self.width))

        self.map_img = np.full_like(grid_2d, 127, dtype=np.uint8)
        self.map_img[grid_2d == 0] = 255
        self.map_img[grid_2d == 100] = 0
        self.map_img = np.flipud(self.map_img)

        self.map_received = True
        rospy.loginfo(f"Map processed: Resolution={self.resolution}, Origin={self.origin}, Size=({self.width}x{self.height})")
        rospy.loginfo("RRT Node Ready for Start/Goal.")

    def start_goal_callback(self, msg):
        if not self.map_received:
            rospy.logwarn("Map not received yet. Cannot process start/goal.")
            return

        if len(msg.data) != 4:
            rospy.logerr(f"Received invalid start/goal message. Expected 4 elements, got {len(msg.data)}")
            return

        x_start_real, y_start_real, x_goal_real, y_goal_real = msg.data
        start_world = [x_start_real, y_start_real]
        goal_world = [x_goal_real, y_goal_real]

        rospy.loginfo(f"Received start: {start_world}, goal: {goal_world}")

        try:
            start_px, goal_px, path_px, path_world = rrt_pathfinder(
                start_world, goal_world, self.map_img, self.resolution, self.origin
            )

            if not path_px:
                rospy.logwarn("RRT failed to find a path.")
                traj_msg = Float64MultiArray()
                traj_msg.data = []
                self.trajectory_pub.publish(traj_msg)
            else:
                rospy.loginfo(f"RRT found path with {len(path_px)} points (pixel coordinates).")
                trajectory_flat = [coord for point in path_px for coord in point]
                traj_msg = Float64MultiArray()
                traj_msg.data = trajectory_flat
                self.trajectory_pub.publish(traj_msg)
                rospy.loginfo("Published trajectory (pixel coordinates).")

                try:
                    map_color = cv2.cvtColor(self.map_img.copy(), cv2.COLOR_GRAY2BGR)

                    for i in range(len(path_px) - 1):
                        pt1 = tuple(map(int, path_px[i]))
                        pt2 = tuple(map(int, path_px[i+1]))
                        cv2.line(map_color, pt1, pt2, (255, 0, 0), 1)

                    if start_px:
                        if len(start_px) == 2 and all(isinstance(p, int) for p in start_px):
                            cv2.circle(map_color, tuple(map(int, start_px)), 3, (0, 255, 0), -1)
                        else:
                            rospy.logwarn(f"Invalid start_px for drawing: {start_px}")
                    if goal_px:
                        if len(goal_px) == 2 and all(isinstance(p, int) for p in goal_px):
                            cv2.circle(map_color, tuple(map(int, goal_px)), 3, (0, 0, 255), -1)
                        else:
                            rospy.logwarn(f"Invalid goal_px for drawing: {goal_px}")

                    start_str = f"s_({start_world[0]:.1f},{start_world[1]:.1f})".replace('.', 'p').replace('-', 'n')
                    goal_str = f"g_({goal_world[0]:.1f},{goal_world[1]:.1f})".replace('.', 'p').replace('-', 'n')
                    filename = f"rrt_path_{start_str}_to_{goal_str}.png"
                    home_dir = os.path.expanduser("~")
                    save_path = os.path.join(home_dir, filename)

                    cv2.imwrite(save_path, map_color)
                    rospy.loginfo(f"Saved trajectory visualization to: {save_path}")

                except Exception as viz_e:
                    rospy.logerr(f"Error during visualization/saving: {viz_e}")

        except Exception as e:
            rospy.logerr(f"Error during RRT planning: {e}")
            traj_msg = Float64MultiArray()
            traj_msg.data = []
            self.trajectory_pub.publish(traj_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        rrt_node = RRTNode()
        rrt_node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception in RRT node: {e}")
        sys.exit(1)
