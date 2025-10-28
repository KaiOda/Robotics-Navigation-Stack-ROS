#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from nav_msgs.msg import Odometry
from tf import transformations
import math

class MotionPlanner:
    def __init__(self):
        rospy.init_node('motion_planner_node', anonymous=True)

        self.start_goal_pub = rospy.Publisher(
            '/start_goal', Float64MultiArray, queue_size=10)

        self.reference_pose_pub = rospy.Publisher(
            '/reference_pose', Float64MultiArray, queue_size=10)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.trajectory_sub = rospy.Subscriber(
            '/trajectory', Float64MultiArray, self.trajectory_callback)

        self.current_pose = None
        self.current_orientation_yaw = None
        self.trajectory_points = []
        self.current_waypoint_index = 0
        self.goal_reached_threshold = 0.15
        self.pid_mode = 1

        rospy.loginfo("Motion Planner Node Initialized. Waiting for /odom...")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        quat = [ori.x, ori.y, ori.z, ori.w]
        _, _, yaw = transformations.euler_from_quaternion(quat)
        self.current_pose = pos
        self.current_orientation_yaw = yaw

        if not self.trajectory_points:
            return

        target = self.trajectory_points[self.current_waypoint_index]
        dx = target[0] - pos.x
        dy = target[1] - pos.y
        dist = math.hypot(dx, dy)

        if dist < self.goal_reached_threshold:
            rospy.loginfo(
                f"Waypoint {self.current_waypoint_index+1}/{len(self.trajectory_points)} reached: {target}")
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.trajectory_points):
                self.publish_next_waypoint()
            else:
                rospy.loginfo("Final goal reached!")
                self.trajectory_points = []

    def trajectory_callback(self, msg):
        rospy.loginfo("Received trajectory from RRT.")
        data = msg.data
        if not data or len(data) % 2 != 0:
            rospy.logwarn("Empty or malformed trajectory received.")
            self.trajectory_points = []
            return

        self.trajectory_points = [
            [data[i], data[i+1]] for i in range(0, len(data), 2)
        ]
        rospy.loginfo(f"Loaded {len(self.trajectory_points)} waypoints.")
        self.current_waypoint_index = 0
        self.publish_next_waypoint()

    def publish_next_waypoint(self):
        if self.current_pose is None:
            rospy.logwarn("Current pose unknown, cannot publish waypoint.")
            return
        if self.current_waypoint_index >= len(self.trajectory_points):
            rospy.logwarn("No more waypoints to publish.")
            return

        xr, yr = self.trajectory_points[self.current_waypoint_index]
        if self.current_waypoint_index < len(self.trajectory_points) - 1:
            nx, ny = self.trajectory_points[self.current_waypoint_index + 1]
            theta_r = math.atan2(ny - yr, nx - xr)
        else:
            theta_r = math.atan2(
                yr - self.current_pose.y, xr - self.current_pose.x)

        ref_msg = Float64MultiArray()
        ref_msg.data = [xr, yr, theta_r, float(self.pid_mode)]
        self.reference_pose_pub.publish(ref_msg)
        rospy.loginfo(
            f"Published waypoint {self.current_waypoint_index+1}/{len(self.trajectory_points)}: "
            f"[{xr:.2f}, {yr:.2f}, {theta_r:.2f}], mode={self.pid_mode}")

    def get_goal_and_plan(self):
        while not self.current_pose and not rospy.is_shutdown():
            rospy.sleep(0.1)
        if rospy.is_shutdown():
            return

        rospy.loginfo(
            f"Current pose: x={self.current_pose.x:.2f}, y={self.current_pose.y:.2f}, yaw={self.current_orientation_yaw:.2f}")
        try:
            xg = float(input("Enter target X coordinate (global): "))
            yg = float(input("Enter target Y coordinate (global): "))
        except ValueError:
            rospy.logerr("Invalid input; please enter numeric values.")
            return

        rospy.loginfo(f"Planning path to ({xg}, {yg})...")
        msg = Float64MultiArray()
        msg.data = [
            self.current_pose.x,
            self.current_pose.y,
            xg,
            yg
        ]
        self.start_goal_pub.publish(msg)
        rospy.loginfo(
            f"Published start/goal: [{self.current_pose.x:.2f}, {self.current_pose.y:.2f}, {xg:.2f}, {yg:.2f}]")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.trajectory_points:
                self.get_goal_and_plan()
            rate.sleep()

if __name__ == '__main__':
    try:
        planner = MotionPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Motion Planner shutting down.")
