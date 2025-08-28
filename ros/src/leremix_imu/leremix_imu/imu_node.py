#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import time
import board
import busio
from adafruit_icm20x import ICM20948

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # Initialize I2C and ICM20948
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.icm = ICM20948(self.i2c)
            self.get_logger().info('ICM20948 initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ICM20948: {str(e)}')
            return
        
        # Create publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        
        # Create timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu_data)
        
        self.get_logger().info(f'IMU node started, publishing at {self.publish_rate} Hz')
    
    def publish_imu_data(self):
        try:
            # Read sensor data
            accel_x, accel_y, accel_z = self.icm.acceleration
            gyro_x, gyro_y, gyro_z = self.icm.gyro
            mag_x, mag_y, mag_z = self.icm.magnetic
            
            # Get current timestamp
            now = self.get_clock().now()
            
            # Create and publish IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = now.to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # Linear acceleration (m/s^2)
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z
            
            # Angular velocity (rad/s)
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            
            # Orientation is not provided by this sensor, set covariance to -1
            imu_msg.orientation_covariance[0] = -1.0
            
            # Set covariance matrices (these would need to be calibrated for your specific sensor)
            # For now, using placeholder values
            for i in range(9):
                imu_msg.linear_acceleration_covariance[i] = 0.0
                imu_msg.angular_velocity_covariance[i] = 0.0
            
            # Set diagonal elements to reasonable values
            imu_msg.linear_acceleration_covariance[0] = 0.01  # x
            imu_msg.linear_acceleration_covariance[4] = 0.01  # y
            imu_msg.linear_acceleration_covariance[8] = 0.01  # z
            
            imu_msg.angular_velocity_covariance[0] = 0.01  # x
            imu_msg.angular_velocity_covariance[4] = 0.01  # y
            imu_msg.angular_velocity_covariance[8] = 0.01  # z
            
            self.imu_pub.publish(imu_msg)
            
            # Create and publish MagneticField message
            mag_msg = MagneticField()
            mag_msg.header.stamp = now.to_msg()
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = mag_x
            mag_msg.magnetic_field.y = mag_y
            mag_msg.magnetic_field.z = mag_z
            
            # Set covariance matrix for magnetometer (placeholder values)
            for i in range(9):
                mag_msg.magnetic_field_covariance[i] = 0.0
            mag_msg.magnetic_field_covariance[0] = 0.01  # x
            mag_msg.magnetic_field_covariance[4] = 0.01  # y
            mag_msg.magnetic_field_covariance[8] = 0.01  # z
            
            self.mag_pub.publish(mag_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error reading IMU data: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    imu_node = IMUNode()
    
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()