#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

current_pose = None

def pose_cb(msg):
    global current_pose
    current_pose = msg

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def swim_to(x_goal, y_goal, pub):
    rate = rospy.Rate(20)
    K_linear = 1.5
    K_angular = 6.0
    distance_tolerance = 0.1
    cmd = Twist()

    while not rospy.is_shutdown():
        if current_pose is None:
            rospy.logwarn_throttle(1.0, "Waiting for current pose...")
            rate.sleep()
            continue

        delta_x = x_goal - current_pose.x
        delta_y = y_goal - current_pose.y
        distance_to_goal = math.sqrt(delta_x**2 + delta_y**2)

        if distance_to_goal < distance_tolerance:
            rospy.loginfo("Goal reached!")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            pub.publish(cmd)
            break

        angle_to_goal = math.atan2(delta_y, delta_x)
        angle_error = normalize_angle(angle_to_goal - current_pose.theta)
        cmd.angular.z = K_angular * angle_error
        alignment_factor = max(0.1, math.cos(angle_error))
        cmd.linear.x = min(K_linear * distance_to_goal * alignment_factor, 2.0)
        pub.publish(cmd)
        rate.sleep()

def main():
    rospy.init_node('swim_to_goal', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, pose_cb)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Waiting for initial pose message...")
    while current_pose is None and not rospy.is_shutdown():
        try:
            time.sleep(0.1)
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutdown requested while waiting for pose.")
            return
    rospy.loginfo("Initial pose received. swim_to_goal node ready.")
    rospy.loginfo("Enter goal coordinates (approx. 0 to 11 for default window).")

    while not rospy.is_shutdown():
        try:
            x_input = input("Enter target x coordinate: ")
            if x_input.lower() in ['q', 'quit', 'exit']:
                break
            x_goal = float(x_input)

            y_input = input("Enter target y coordinate: ")
            if y_input.lower() in ['q', 'quit', 'exit']:
                break
            y_goal = float(y_input)

        except ValueError:
            rospy.logwarn("Invalid input. Please enter numeric coordinates or 'q' to quit.")
            continue
        except EOFError:
            rospy.loginfo("EOF received, shutting down.")
            break

        rospy.loginfo(f"Moving to goal: ({x_goal:.2f}, {y_goal:.2f})")
        swim_to(x_goal, y_goal, pub)
        if rospy.is_shutdown():
            rospy.loginfo("Shutdown requested during movement.")
            break

    rospy.loginfo("swim_to_goal node shutting down.")
    stop_cmd = Twist()
    pub.publish(stop_cmd)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received. Shutting down.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")
    finally:
        try:
            if 'pub' in globals() and pub is not None:
                stop_cmd = Twist()
                pub.publish(stop_cmd)
        except NameError:
            pass
        except Exception as final_e:
            rospy.logerr(f"Error during final cleanup: {final_e}")

