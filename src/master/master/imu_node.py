#!/usr/bin/env python3
import math
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

CALIB_SAMPLES = 200   # collect 50 packets at startup (~5 sec at 10Hz)


def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    return Quaternion(
        x = sr*cp*cy - cr*sp*sy,
        y = cr*sp*cy + sr*cp*sy,
        z = cr*cp*sy - sr*sp*cy,
        w = cr*cp*cy + sr*sp*sy
    )


def get_acc(d):
    k = 16.0
    def s(h, l): v = (h<<8|l)/32768.0*k; return v - 2*k if v >= k else v
    return s(d[1],d[0])*9.81, s(d[3],d[2])*9.81, s(d[5],d[4])*9.81

def get_gyro(d):
    k = 2000.0
    def s(h, l): v = (h<<8|l)/32768.0*k; return v - 2*k if v >= k else v
    return math.radians(s(d[1],d[0])), math.radians(s(d[3],d[2])), math.radians(s(d[5],d[4]))

def get_angle(d):
    k = 180.0
    def s(h, l): v = (h<<8|l)/32768.0*k; return v - 2*k if v >= k else v
    return s(d[1],d[0]), s(d[3],d[2]), s(d[5],d[4])


class CMP10ANode(Node):
    def __init__(self):
        super().__init__('cmp10a_imu')
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=0.01)
        self.get_logger().info("CMP10A IMU node started — keep sensor FLAT and STILL for calibration...")

        # Parser state
        self.buf       = []
        self.start     = 0
        self.checksum  = 0
        self.data_left = 0

        # Latest raw values
        self.acc   = (0.0, 0.0, 9.81)
        self.gyro  = (0.0, 0.0, 0.0)
        self.angle = (0.0, 0.0, 0.0)

        # Calibration offsets
        self.calib_roll  = 0.0
        self.calib_pitch = 0.0
        self.calib_yaw   = 0.0
        self.calib_gx    = 0.0
        self.calib_gy    = 0.0
        self.calib_gz    = 0.0

        # Calibration state
        self.calibrated    = False
        self.calib_samples = []
        self.calib_count   = 0

        self.create_timer(0.005, self.read_serial)  # 200 Hz poll

    def read_serial(self):
        n = self.ser.in_waiting
        if n == 0:
            return
        for byte in self.ser.read(n):
            self.due_data(byte)

    def due_data(self, b):
        if b == 0x55 and self.start == 0:
            self.start     = 1
            self.data_left = 11
            self.checksum  = 0
            self.buf       = [0] * 11
        if self.start == 1:
            self.checksum += b
            self.buf[11 - self.data_left] = b
            self.data_left -= 1
            if self.data_left == 0:
                self.checksum = (self.checksum - b) & 0xFF
                self.start    = 0
                self.handle_packet()

    def handle_packet(self):
        if self.buf[10] != self.checksum:
            return
        d = self.buf[2:8]
        t = self.buf[1]
        if   t == 0x51: self.acc   = get_acc(d)
        elif t == 0x52: self.gyro  = get_gyro(d)
        elif t == 0x53:
            self.angle = get_angle(d)
            self.process_angle_packet()

    def process_angle_packet(self):
        if not self.calibrated:
            self.calib_samples.append((self.angle, self.gyro))
            self.calib_count += 1
            self.get_logger().info(
                f"Calibrating... {self.calib_count}/{CALIB_SAMPLES}",
                throttle_duration_sec=1.0)

            if self.calib_count >= CALIB_SAMPLES:
                self.calib_roll  = sum(s[0][0] for s in self.calib_samples) / CALIB_SAMPLES
                self.calib_pitch = sum(s[0][1] for s in self.calib_samples) / CALIB_SAMPLES
                self.calib_yaw   = sum(s[0][2] for s in self.calib_samples) / CALIB_SAMPLES
                self.calib_gx    = sum(s[1][0] for s in self.calib_samples) / CALIB_SAMPLES
                self.calib_gy    = sum(s[1][1] for s in self.calib_samples) / CALIB_SAMPLES
                self.calib_gz    = sum(s[1][2] for s in self.calib_samples) / CALIB_SAMPLES
                self.calibrated  = True
                self.get_logger().info(
                    f"Calibration done! Offsets — "
                    f"roll:{self.calib_roll:.2f} "
                    f"pitch:{self.calib_pitch:.2f} "
                    f"yaw:{self.calib_yaw:.2f} deg"
                )
        else:
            self.publish_imu()

    def publish_imu(self):
        roll  = self.angle[0] - self.calib_roll
        pitch = self.angle[1] - self.calib_pitch
        yaw   = self.angle[2] - self.calib_yaw
        gx    = self.gyro[0]  - self.calib_gx
        gy    = self.gyro[1]  - self.calib_gy
        gz    = self.gyro[2]  - self.calib_gz

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation     = euler_to_quaternion(roll, pitch, yaw)
        msg.angular_velocity.x    = gx
        msg.angular_velocity.y    = gy
        msg.angular_velocity.z    = gz
        msg.linear_acceleration.x = self.acc[0]
        msg.linear_acceleration.y = self.acc[1]
        msg.linear_acceleration.z = self.acc[2]
        msg.orientation_covariance[0]         = -1.0
        msg.angular_velocity_covariance[0]    = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CMP10ANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
