#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np

# Zones: [left-far, left, centre, right, right-far]
ZONE_NAMES = ['left_far', 'left', 'centre', 'right', 'right_far']
MIN_DEPTH = 0.3   # metres — ignore closer (noise)
MAX_DEPTH = 4.0   # metres — ignore farther (not urgent)

class ObstacleZoneNode(Node):
    def __init__(self):
        super().__init__('obstacle_zone_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw',
            self.depth_callback, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/obstacles/zones', 10)
        self.get_logger().info("Obstacle zone node started")

    def depth_callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth = depth.astype(np.float32) * 0.001  # mm → metres

        h, w = depth.shape
        # Ignore top 30% (sky/ceiling) and bottom 15% (floor)
        roi = depth[int(h*0.15):int(h*0.70), :]

        # Divide width into 5 equal zones
        zones = np.array_split(roi, 5, axis=1)
        distances = []
        for zone in zones:
            valid = zone[(zone > MIN_DEPTH) & (zone < MAX_DEPTH)]
            dist = float(np.percentile(valid, 5)) if len(valid) > 0 else MAX_DEPTH
            distances.append(dist)

        msg_out = Float32MultiArray()
        msg_out.data = distances
        self.pub.publish(msg_out)

        # Debug log
        info = ' | '.join(f'{n}:{d:.2f}m' for n, d in zip(ZONE_NAMES, distances))
        self.get_logger().info(info, throttle_duration_sec=0.5)

def main():
    rclpy.init()
    rclpy.spin(ObstacleZoneNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

