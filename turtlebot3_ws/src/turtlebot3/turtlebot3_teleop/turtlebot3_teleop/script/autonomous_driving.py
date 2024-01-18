import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class Turtlebot3Drive(Node):
    def __init__(self):
        super().__init__('turtlebot3_autonomous_driving')
        self.scan_data = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.prev_yaw = 0.0
        self.escape_range = 15 * math.pi / 180.0
        self.max_forward_dist = 0.5
        self.max_side_dist = 0.4
        self.turtlebot3_state = 0  # 0 - получить направление, 1 - ехать вперед, 2 - повернуть направо, 3 - повернуть налево
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.update_timer = self.create_timer(0.01, self.update_callback)

    def euler_from_quaternion(self, x, y, z, w):
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, self.yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

    def scan_callback(self, msg):
        scan_angle = [0, 30, 330]  # 0 - впереди, 30 - справа, 330 - слева
        for num in range(3):
            if math.isinf(msg.ranges[scan_angle[num]]):
                self.scan_data[num] = msg.range_max
            else:
                self.scan_data[num] = msg.ranges[scan_angle[num]]

    def update_cmd_vel(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.cmd_vel_pub.publish(cmd_vel)

    def update_callback(self):
        if self.turtlebot3_state == 0: 
            if self.scan_data[0] > self.max_forward_dist:
                if self.scan_data[2] < self.max_side_dist:
                    self.prev_yaw = self.yaw
                    self.turtlebot3_state = 3  
                elif self.scan_data[1] < self.max_side_dist:
                    self.prev_yaw = self.yaw
                    self.turtlebot3_state = 2  
                else:
                    self.turtlebot3_state = 1                  
            else:
                self.prev_yaw = self.yaw
                self.turtlebot3_state = 2  

        if self.turtlebot3_state == 1: # вперед
            self.update_cmd_vel(0.2, 0.0)
            self.turtlebot3_state = 0  

        if self.turtlebot3_state == 2:  # направо
            if abs(self.prev_yaw - self.yaw) >= self.escape_range:
                self.turtlebot3_state = 0
            else:
                self.update_cmd_vel(0.0, -0.2)

        if self.turtlebot3_state == 3:  # налево
            if abs(self.prev_yaw - self.yaw) >= self.escape_range:
                self.turtlebot3_state = 0 
            else:
                self.update_cmd_vel(0.0, 0.2)


if __name__ == '__main__':
    rclpy.init(args=None)
    turtlebot3_drive = Turtlebot3Drive()
    rclpy.spin(turtlebot3_drive)
    turtlebot3_drive.destroy_node()
    rclpy.shutdown()