#!/usr/bin/env python3
import rospy, math
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

# tolerances (must match controller)
POS_TOL = 0.1
ANG_TOL = 0.1

current_pose = None
goal_active  = False

def odom_cb(msg):
    global current_pose, goal_active
    p = msg.pose.pose
    current_pose = (p.position.x, p.position.y, 
                    math.atan2(2*(p.orientation.w*p.orientation.z + p.orientation.x*p.orientation.y),
                               1 - 2*(p.orientation.y**2 + p.orientation.z**2)))
    # only check final goal completion here
    if goal_active and target is not None:
        tx, ty, tt, _ = target
        cx, cy, ct  = current_pose
        if math.hypot(tx-cx, ty-cy) < POS_TOL and abs(_angle_diff(tt, ct)) < ANG_TOL:
            rospy.loginfo("Reached! pos err={:.3f}, ang err={:.3f}".format(
                math.hypot(tx-cx, ty-cy), abs(_angle_diff(tt, ct))))
            rospy.sleep(0.5)  
            goal_active = False

def _angle_diff(a,b):
    d = a-b
    return (d + math.pi) % (2*math.pi) - math.pi

if __name__=='__main__':
    rospy.init_node('motion_planner')
    pub = rospy.Publisher('/reference_pose', Float32MultiArray, queue_size=1)
    rospy.Subscriber('/odom', Odometry, odom_cb)
    rospy.loginfo("Waiting for odometry...")
    rospy.wait_for_message('/odom', Odometry, timeout=5.0)
    rospy.loginfo("Ready. Enter goals in **radians**:")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if not goal_active:
            try:
                vals = input("x y theta_rad mode(0|1)> ").split()
                if len(vals)!=4:
                    raise ValueError
                x, y, th, m = float(vals[0]), float(vals[1]), float(vals[2]), int(vals[3])
                if m not in (0,1):
                    raise ValueError
            except ValueError:
                rospy.logwarn("Invalid. Format: x y theta_rad mode(0 or 1)")
                continue

            target = (x, y, th, m)
            arr = Float32MultiArray(data=[x, y, th, m])
            pub.publish(arr)
            goal_active = True
            rospy.loginfo(f"Sent: x={x}, y={y}, θ={th} rad, mode={m}")
            rospy.loginfo("Waiting for arrival...")

        rate.sleep()
