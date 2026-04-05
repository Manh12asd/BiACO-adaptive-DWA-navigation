#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
DEVICE_ADDRESS = 0x68

class MPU6050_Driver(Node):

    def __init__(self):
        super().__init__("mpu6050_driver")
        
        # --- Thông số Offset đã hiệu chuẩn ---
        self.A_OFF_X = 816
        self.A_OFF_Y = -293
        self.A_OFF_Z = 1123
        self.G_OFF_X = 36
        self.G_OFF_Y = 10
        self.G_OFF_Z = -44
        
        # I2C Interface
        self.is_connected_ = False
        self.init_i2c()

        # ROS 2 Interface - Đã đổi QoS sang 10 (Reliable) cho tương thích EKF
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", 10)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "imu_link"
        self.frequency_ = 0.01  # Tương đương 100Hz
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()
            
            # 1. Đọc dữ liệu thô (Raw)
            raw_acc_x = self.read_raw_data(ACCEL_XOUT_H)
            raw_acc_y = self.read_raw_data(ACCEL_YOUT_H)
            raw_acc_z = self.read_raw_data(ACCEL_ZOUT_H)
            
            raw_gyro_x = self.read_raw_data(GYRO_XOUT_H)
            raw_gyro_y = self.read_raw_data(GYRO_YOUT_H)
            raw_gyro_z = self.read_raw_data(GYRO_ZOUT_H)
            
            # 2. Trừ Offset để khử nhiễu drift
            acc_x = raw_acc_x - self.A_OFF_X
            acc_y = raw_acc_y - self.A_OFF_Y
            acc_z = raw_acc_z - self.A_OFF_Z
            
            gyro_x = raw_gyro_x - self.G_OFF_X
            gyro_y = raw_gyro_y - self.G_OFF_Y
            gyro_z = raw_gyro_z - self.G_OFF_Z
            
            # 3. Chuyển đổi sang đơn vị chuẩn (m/s^2 và rad/s)
            self.imu_msg_.linear_acceleration.x = acc_x / 1670.13
            self.imu_msg_.linear_acceleration.y = acc_y / 1670.13
            self.imu_msg_.linear_acceleration.z = acc_z / 1670.13
            
            # Hệ số chia đã đổi thành 469.82 để khớp góc quay thực tế
            self.imu_msg_.angular_velocity.x = gyro_x / 939.65
            self.imu_msg_.angular_velocity.y = gyro_y / 939.65
            self.imu_msg_.angular_velocity.z = gyro_z / 939.65

            # 4. Thêm Ma trận Hiệp phương sai (Covariance) cho EKF
            # Báo cho EKF bỏ qua góc xoay tuyệt đối (chỉ dùng vận tốc)
            self.imu_msg_.orientation_covariance[0] = -1.0
            
            # Khai báo độ nhiễu nhỏ để EKF có thể tính toán
            self.imu_msg_.linear_acceleration_covariance[0] = 0.01
            self.imu_msg_.linear_acceleration_covariance[4] = 0.01
            self.imu_msg_.linear_acceleration_covariance[8] = 0.01
            
            self.imu_msg_.angular_velocity_covariance[0] = 0.01
            self.imu_msg_.angular_velocity_covariance[4] = 0.01
            self.imu_msg_.angular_velocity_covariance[8] = 0.01

            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)
            
        except OSError:
            self.is_connected_ = False

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)
            # Cấu hình Gyro ở dải +/- 2000 deg/s
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 24)
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)
            self.is_connected_ = True
        except OSError:
            self.is_connected_ = False
        
    def read_raw_data(self, addr):
        #Accelero and Gyro value are 16-bit
        high = self.bus_.read_byte_data(DEVICE_ADDRESS, addr)
        low = self.bus_.read_byte_data(DEVICE_ADDRESS, addr+1)
        
        #concatenate higher and lower value
        value = ((high << 8) | low)
            
        #to get signed value from mpu6050
        if(value > 32768):
            value = value - 65536
        return value

def main():
    rclpy.init()
    mpu6050_driver = MPU6050_Driver()
    rclpy.spin(mpu6050_driver)
    mpu6050_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
