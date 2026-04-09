import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf_transformations

import serial
import threading
import struct
import math
import time
from enum import Enum

class CMD(Enum):
    IMU = 0
    ODOM = 1
    CMD_VEL = 2
    
wheel_radius = 0.0325
wheel_sep = 0.19
encoder_res = 11 * 46.8

class MyrobotManager(Node):
    def __init__(self):
        super().__init__('myrobot_manager')
        
        self.serial = serial.Serial("/dev/ttyAMA0", 115200, timeout=0.01)
        
        threading.Thread(target=self.read_data, daemon=True).start()
        
        self.gyro = [0.0, 0.0, 0.0]
        self.accel = [0.0, 0.0, 0.0]
        self.encoder = [0, 0]
        self.last_time = self.get_clock().now()
        self.left_last = 0
        self.right_last = 0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.imu_publisher = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.imu_publish_timer = self.create_timer(0.02, self.publish_imu)
        self.odom_publish_timer = self.create_timer(0.03, self.publish_odom)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.imu_msg = None
        self.odom_msg = None
        
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z / 5
        
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z
        self.get_logger().info(f"left_speed : {left_speed}, right_speed : {right_speed}")
        set_l = self.custom_map(left_speed)
        set_r = self.custom_map(right_speed)
        data = struct.pack("<BBffB", 0xaa, CMD.CMD_VEL.value, set_l, set_r, 0xff)
        
        self.serial.write(data)
        self.serial.flush()
        
    def custom_map(self, value):
        if value == 0:
            return 0
        if value > 0:
            result = 30 + ((value * 20) / 0.5)
        else:
            result = -30 + ((value * 20) / 0.5)
        return result
    
    def read_data(self):
        buffer = bytearray()
        while True:
            data = self.serial.read(1)
            if data == b'\xaa':
                buffer = bytearray()
            elif data == b'\xff':
                if len(buffer) < 2:
                    continue
                if buffer[0] == CMD.IMU.value:
                    if (len(buffer) < 25):
                        continue
                    imu = struct.unpack('<6f', buffer[1:25])
                    self.gyro[0:3] = imu[0:3]
                    self.accel[0:3] = imu[3:6]
                    self.update_imu()
                elif buffer[0] == CMD.ODOM.value:
                    if (len(buffer) < 9):
                        continue
                    self.encoder = struct.unpack('<2i', buffer[1:9])
                    self.update_odom()
            else:
                buffer += data
    
    def update_imu(self):
        self.imu_msg = Imu()
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.header.frame_id = "imu_link"
        self.imu_msg.angular_velocity.x = self.gyro[0]
        self.imu_msg.angular_velocity.y = self.gyro[1]
        self.imu_msg.angular_velocity.z = self.gyro[2]
        
        self.imu_msg.linear_acceleration.x = self.accel[0]
        self.imu_msg.linear_acceleration.y = self.accel[1]
        self.imu_msg.linear_acceleration.z = self.accel[2]
        
        self.imu_msg.orientation_covariance = [9999999, 0, 0,
                                          0, 9999999, 0,
                                          0, 0, 0.01]
        self.imu_msg.angular_velocity_covariance = [0.02, 0, 0,
                                               0, 0.02, 0,
                                               0, 0, 0.01]
        self.imu_msg.linear_acceleration_covariance = [0.04, 0, 0,
                                                  0, 0.04, 0,
                                                  0, 0, 0.04]
        
    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        delta_left = self.encoder[0] - self.left_last
        delta_right = self.encoder[1] - self.right_last
        self.left_last = self.encoder[0]
        self.right_last = self.encoder[1]

        d_left = (2*math.pi*wheel_radius*delta_left)/encoder_res
        d_right = (2*math.pi*wheel_radius*delta_right)/encoder_res

        d_center = (d_left + d_right)/2
        d_theta = (d_right - d_left)/wheel_sep

        self.x += d_center * math.cos(self.th + d_theta/2)
        self.y += d_center * math.sin(self.th + d_theta/2)
        self.th += d_theta

        self.odom_msg = Odometry()
        self.odom_msg.header.stamp = now.to_msg()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0
        
        q = tf_transformations.quaternion_from_euler(0,0,self.th)
        
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]

        self.odom_msg.twist.twist.linear.x = d_center/dt
        self.odom_msg.twist.twist.angular.z = d_theta/dt
    
    def publish_imu(self):
        with self.lock:
            msg = self.imu_msg
        if self.imu_msg is not None:
            self.imu_publisher.publish(msg)
    
    def publish_odom(self):
        with self.lock:
            msg = self.imu_msg
        if self.odom_msg is not None:
            self.odom_publisher.publish(msg)
    
    def __del__(self):
        self.serial.close()
        
def main(args=None):
    rclpy.init(args=args)
    node = MyrobotManager()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()