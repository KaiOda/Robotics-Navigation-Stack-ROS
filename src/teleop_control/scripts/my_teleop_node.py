#!/usr/bin/env python3
import rospy
import curses
from geometry_msgs.msg import Twist

def main(stdscr):
    # Initialize ROS node
    rospy.init_node('my_teleop_node', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Initialize curses for keyboard input
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)
    
    # Instructions for the user
    stdscr.addstr(0, 0, "Press 'w' to move forward, 's' to move backward,")
    stdscr.addstr(1, 0, "'a' to rotate left, 'd' to rotate right, and 'q' to quit.")
    
    try:
        while not rospy.is_shutdown():
            key = stdscr.getch()   # Get user key press

            # Create a new Twist message
            move_cmd = Twist()
            
            # Set default speeds to zero
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            
            if key == ord('w'):
                move_cmd.linear.x = 0.3  # Forward
            elif key == ord('s'):
                move_cmd.linear.x = -0.3 # Backward
            elif key == ord('a'):
                move_cmd.angular.z = 0.5 # Rotate counterclockwise
            elif key == ord('d'):
                move_cmd.angular.z = -0.5 # Rotate clockwise
            elif key == ord('q'):
                break  # Exit on 'q'
            
            # Only send the message if a movement key was pressed
            if move_cmd.linear.x != 0.0 or move_cmd.angular.z != 0.0:
                pub.publish(move_cmd)
            
            rate.sleep()
    finally:
        # Restore terminal settings on exit
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()

if __name__ == '__main__':
    # Wrap the main function in curses.wrapper to ensure proper
    # initialization and cleanup of curses
    curses.wrapper(main)
