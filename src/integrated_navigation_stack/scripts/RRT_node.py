#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rrt import rrt_pathfinder

map_img_global = None
map_resolution_global = None
map_origin_global = None

def occupancy_grid_callback(msg):
    global map_img_global, map_resolution_global, map_origin_global

    map_resolution_global = msg.info.resolution
    map_origin_global = [msg.info.origin.position.x, msg.info.origin.position.y]
    width = msg.info.width
    height = msg.info.height
    data = np.array(msg.data).reshape((height, width))

    img = np.full_like(data, 127, dtype=np.uint8)
    img[data == 0] = 255
    img[data == 100] = 0

    map_img_global = np.flipud(img)
    rospy.loginfo("RRT Node: Map processed and stored.")

def start_goal_callback(msg, trajectory_pub_arg):
    global map_img_global, map_resolution_global, map_origin_global

    if map_img_global is None or map_resolution_global is None or map_origin_global is None:
        rospy.logwarn("RRT Node: Map not received yet. Cannot plan.")
        return

    if len(msg.data) != 4:
        rospy.logwarn(f"RRT Node: Received invalid /start_goal data length: {len(msg.data)}. Expected 4.")
        return

    x_start_real, y_start_real, x_goal_real, y_goal_real = msg.data
    start_world = [x_start_real, y_start_real]
    goal_world = [x_goal_real, y_goal_real]

    rospy.loginfo(f"RRT Node: Received planning request - Start: {start_world}, Goal: {goal_world}")

    _, _, _, path_world = rrt_pathfinder(
        start_world, goal_world,
        map_img_global, map_resolution_global, map_origin_global
    )

    if not path_world:
        rospy.logwarn("RRT Node: RRT failed to find a path or path is empty.")
        trajectory_msg = Float64MultiArray()
        trajectory_msg.data = []
        trajectory_pub_arg.publish(trajectory_msg)
        return

    rospy.loginfo(f"RRT Node: Path found with {len(path_world)} waypoints.")

    trajectory_data = []
    for point in path_world:
        trajectory_data.append(point[0])
        trajectory_data.append(point[1])

    trajectory_msg = Float64MultiArray()
    trajectory_msg.data = trajectory_data
    trajectory_pub_arg.publish(trajectory_msg)
    rospy.loginfo("RRT Node: Published trajectory to /trajectory topic.")

def rrt_node_main():
    rospy.init_node('rrt_planning_node', anonymous=True)
    rospy.loginfo("RRT Planning Node Started.")

    trajectory_pub = rospy.Publisher('/trajectory', Float64MultiArray, queue_size=10)
    rospy.Subscriber('/map', OccupancyGrid, occupancy_grid_callback)
    rospy.Subscriber('/start_goal', Float64MultiArray,
                     lambda msg: start_goal_callback(msg, trajectory_pub))

    rospy.loginfo("RRT Node spinning, waiting for map and start/goal requests.")
    rospy.spin()

if __name__ == '__main__':
    try:
        rrt_node_main()
    except rospy.ROSInterruptException:
        pass
