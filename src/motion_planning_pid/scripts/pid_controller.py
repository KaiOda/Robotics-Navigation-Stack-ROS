#!/usr/bin/env python3
import rospy, math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PID:
    def __init__(self, Kp, Ki, Kd, out_min, out_max, int_max):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.out_min, self.out_max = out_min, out_max
        self.int_max = abs(int_max)
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = rospy.Time.now()

    def compute(self, error):
        now = rospy.Time.now()
        dt = (now - self.prev_time).to_sec()
        if dt <= 0.0:
            return 0.0
        self.integral = max(-self.int_max, min(self.int_max, self.integral + error * dt))
        derivative = (error - self.prev_error) / dt
        out = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
        out = max(self.out_min, min(self.out_max, out))
        self.prev_error, self.prev_time = error, now
        return out

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = rospy.Time.now()

class PIDControllerNode:
    def __init__(self):
        rospy.init_node('pid_controller')

        # gains & limits from params
        lin_g = rospy.get_param('~linear_gains',  [0.5, 0.01, 0.1])
        ang_g = rospy.get_param('~angular_gains', [1.0, 0.05, 0.2])
        lin_lim = rospy.get_param('~linear_limit',  [0.0, 0.22])
        ang_lim = rospy.get_param('~angular_limit',[ -2.84, 2.84])
        self.pos_tol   = rospy.get_param('~pos_tolerance',   0.1)
        self.angle_tol = rospy.get_param('~angle_tolerance', 0.1)

        self.lin_pid = PID(*lin_g, lin_lim[0], lin_lim[1], lin_lim[1]/2.0)
        self.ang_pid = PID(*ang_g, ang_lim[0], ang_lim[1], ang_lim[1]/2.0)

        self.target = None   # dict with x,y,theta,mode
        self.pose   = None   # dict with x,y,theta
        self.stage  = 0      # for mode 0 FSM

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/reference_pose', Float32MultiArray, self.ref_cb)
        rospy.Subscriber('/odom',           Odometry,            self.odom_cb)

        rospy.on_shutdown(self.stop)
        rospy.spin()

    def stop(self):
        self.pub.publish(Twist())

    def ref_cb(self, msg):
        data = msg.data
        if len(data) != 4 or int(data[3]) not in (0,1):
            rospy.logwarn(f"Ignoring bad reference data: {data}")
            return
        self.target = {
            'x':     data[0],
            'y':     data[1],
            'theta': data[2],
            'mode':  int(data[3])
        }
        self.stage = 0
        self.lin_pid.reset()
        self.ang_pid.reset()
        rospy.loginfo(f"New goal: {self.target}")

    def odom_cb(self, odom):
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        yaw = euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
        self.pose = {'x':p.x, 'y':p.y, 'theta':yaw}
        self.control_loop()

    def control_loop(self):
        if not self.target or not self.pose:
            return

        tx, ty, tt, mode = (self.target[k] for k in ('x','y','theta','mode'))
        cx, cy, ct         = (self.pose[k]   for k in ('x','y','theta'))

        dx, dy = tx-cx, ty-cy
        rho    = math.hypot(dx, dy)
        alpha  = self._angle_diff(math.atan2(dy, dx), ct)
        beta   = self._angle_diff(tt, ct)

        cmd = Twist()

        if mode == 0:
            # rotate → move → rotate
            if self.stage == 0:
                if abs(alpha) > self.angle_tol:
                    cmd.angular.z = self.ang_pid.compute(alpha)
                else:
                    self.stage = 1
                    self.lin_pid.reset()
                    self.ang_pid.reset()
            elif self.stage == 1:
                if rho > self.pos_tol:
                    cmd.linear.x  = self.lin_pid.compute(rho)
                else:
                    self.stage = 2
                    self.ang_pid.reset()
            else:  # stage 2
                if abs(beta) > self.angle_tol:
                    cmd.angular.z = self.ang_pid.compute(beta)
                else:
                    rospy.loginfo("Mode 0: goal reached")
                    self.target = None

        else:  # mode 1
            if rho > self.pos_tol:
                cmd.linear.x  = self.lin_pid.compute(rho)
                cmd.angular.z = self.ang_pid.compute(alpha)
            else:
                if abs(beta) > self.angle_tol:
                    cmd.angular.z = self.ang_pid.compute(beta)
                else:
                    rospy.loginfo("Mode 1: goal reached")
                    self.target = None

        self.pub.publish(cmd)

    def _angle_diff(self, a, b):
        d = a - b
        return (d + math.pi) % (2*math.pi) - math.pi

if __name__=='__main__':
    PIDControllerNode()
