#!/usr/bin/env python3
import rospy
import math
import time
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportAbsoluteRequest
from std_srvs.srv import Empty, EmptyRequest

def swim_vertical_figure_eight():
    rospy.init_node('swim_node_vertical_figure_eight', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(30)

    rospy.loginfo("Waiting for ROS services...")
    try:
        rospy.wait_for_service('/reset', timeout=5.0)
        rospy.wait_for_service('/turtle1/teleport_absolute', timeout=5.0)
        reset_sim = rospy.ServiceProxy('/reset', Empty)
        teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        rospy.loginfo("Services [/reset, /turtle1/teleport_absolute] found.")
    except (rospy.ServiceException, rospy.exceptions.ROSException, rospy.exceptions.ROSInterruptException) as e:
        rospy.logerr(f"Failed to connect to required services: {e}")
        return

    rospy.loginfo("Resetting simulation and positioning turtle...")
    try:
        reset_sim(EmptyRequest())
        rospy.loginfo("Simulation reset.")

        teleport_req = TeleportAbsoluteRequest()
        teleport_req.x = 5.544445
        teleport_req.y = 5.544445
        teleport_req.theta = math.pi / 2.0

        teleport(teleport_req)
        rospy.loginfo(f"Turtle teleported to ({teleport_req.x:.2f}, {teleport_req.y:.2f}) facing UP ({teleport_req.theta:.2f} rad).")

        time.sleep(0.2)

    except (rospy.ServiceException, rospy.exceptions.ROSInterruptException) as e:
        rospy.logerr(f"Service call failed during reset/teleport: {e}")
        return

    linear_vel = 2.0
    angular_amp = 2.0
    period = 10.0
    omega = (2.0 * math.pi) / period

    rospy.loginfo(f"Starting vertical figure 8 motion: LinVel={linear_vel:.2f}, AngAmp={angular_amp:.2f}, Period={period:.1f}s")

    start_t = rospy.Time.now().to_sec()
    cmd = Twist()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        if current_time < start_t:
            rospy.logwarn("ROS time moved backwards, resetting start time.")
            start_t = current_time
        t = current_time - start_t

        cmd.linear.x = linear_vel
        cmd.angular.z = angular_amp * math.sin(omega * t)

        pub.publish(cmd)

        try:
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            rospy.loginfo("ROS Interrupt during sleep.")
            break
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn("ROS time moved backwards during sleep, resetting start time.")
            start_t = rospy.Time.now().to_sec()

if __name__ == '__main__':
    pub_cleanup = None
    try:
        rospy.init_node('swim_node_vertical_figure_eight', anonymous=True)
        pub_cleanup = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
        swim_vertical_figure_eight()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl+C detected. Shutting down node.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred in swim_node: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if pub_cleanup is not None and not rospy.is_shutdown():
            rospy.loginfo("Sending zero velocity command before exiting.")
            final_cmd = Twist()
            pub_cleanup.publish(final_cmd)
            time.sleep(0.1)
