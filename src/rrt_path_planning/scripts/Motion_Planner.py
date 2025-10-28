#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import sys

class MotionPlanner:
    def __init__(self):
        rospy.init_node('motion_planner_node', anonymous=True)
        self.start_goal_pub = rospy.Publisher('/start_goal', Float64MultiArray, queue_size=10)
        rospy.Subscriber('/trajectory', Float64MultiArray, self.trajectory_callback)
        self.last_trajectory = None
        rospy.loginfo("Motion Planner Node Initialized.")

    def trajectory_callback(self, msg):
        rospy.loginfo("Received trajectory from RRT Node.")
        if not msg.data:
            print("\n>>> RRT Algorithm could not find a path. <<<\n")
            self.last_trajectory = []
        else:
            path_pixels = []
            for i in range(0, len(msg.data), 2):
                if i + 1 < len(msg.data):
                    path_pixels.append((int(msg.data[i]), int(msg.data[i+1])))

            print("\n--- Received Trajectory (Pixel Coordinates) ---")
            print(path_pixels)
            print("-----------------------------------------------\n")
            self.last_trajectory = path_pixels

    def get_user_input(self):
        while not rospy.is_shutdown():
            try:
                print("\nEnter Start and Goal Coordinates (in meters, separated by spaces)")
                start_x_str = input("Enter Start X: ")
                start_y_str = input("Enter Start Y: ")
                goal_x_str = input("Enter Goal X: ")
                goal_y_str = input("Enter Goal Y: ")
                start_x = float(start_x_str)
                start_y = float(start_y_str)
                goal_x = float(goal_x_str)
                goal_y = float(goal_y_str)
                return [start_x, start_y, goal_x, goal_y]
            except ValueError:
                print("Invalid input. Please enter numeric values.")
            except EOFError:
                print("\nExiting Motion Planner.")
                sys.exit(0)
            except KeyboardInterrupt:
                print("\nExiting Motion Planner.")
                sys.exit(0)

    def run(self):
        rospy.sleep(1.0)
        while not rospy.is_shutdown():
            start_goal_data = self.get_user_input()
            if start_goal_data:
                msg = Float64MultiArray()
                msg.data = start_goal_data
                self.last_trajectory = None
                rospy.loginfo(f"Publishing Start/Goal: {start_goal_data}")
                self.start_goal_pub.publish(msg)
                rospy.loginfo("Waiting for RRT response...")
                timeout = rospy.Duration(10.0)
                start_time = rospy.Time.now()
                while self.last_trajectory is None and (rospy.Time.now() - start_time) < timeout and not rospy.is_shutdown():
                    rospy.sleep(0.1)
                if self.last_trajectory is None and not rospy.is_shutdown():
                    rospy.logwarn("Timeout waiting for trajectory response.")
                    print("\n>>> No response received from RRT node within timeout. <<<\n")

if __name__ == '__main__':
    try:
        planner = MotionPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        print("Motion Planner interrupted. Exiting.")
    except Exception as e:
        rospy.logerr(f"Unhandled exception in Motion Planner: {e}")
        sys.exit(1)
