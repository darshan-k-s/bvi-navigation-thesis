#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge
import numpy as np
import json
import threading

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

# BVI-relevant classes only — ignore irrelevant COCO objects
BVI_CLASSES = {
    'person', 'bicycle', 'car', 'motorcycle', 'bus', 'truck',
    'chair', 'couch', 'dining table', 'bed',
    'dog', 'cat',
    'traffic light', 'stop sign', 'bench',
    'suitcase', 'backpack',
    'bottle', 'cup', 'bowl',
}

ZONE_NAMES     = ['left_far', 'left', 'centre', 'right', 'right_far']
ZONE_X_OFFSETS = [-0.6, -0.3, 0.0, 0.3, 0.6]

MIN_DEPTH      = 0.3
MAX_DEPTH      = 4.0
CONF_THRESHOLD = 0.45
INFERENCE_HZ   = 3       # RPi-safe rate


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        if not YOLO_AVAILABLE:
            self.get_logger().error(
                "ultralytics not installed — run: pip install ultralytics")
            return

        self.get_logger().info("Loading YOLOv8n (downloads ~6MB on first run)...")
        self.model = YOLO('yolov8n.pt')
        self.model.fuse()
        self.get_logger().info("YOLOv8n ready")

        self.bridge = CvBridge()
        self.lock   = threading.Lock()

        self.rgb_image   = None
        self.depth_image = None

        # RGB — need colour enabled in launch
        self.sub_rgb = self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self.rgb_callback, 5)

        # Aligned depth — same pixel grid as RGB
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback, 5)

        self.pub_detections = self.create_publisher(
            String,      '/obstacles/detections',    10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/obstacles/yolo_markers',  10)

        self.create_timer(1.0 / INFERENCE_HZ, self.run_inference)
        self.get_logger().info(f"YOLO node started at {INFERENCE_HZ} Hz")

    # ── Callbacks ────────────────────────────────────────────
    def rgb_callback(self, msg):
        with self.lock:
            self.rgb_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        with self.lock:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough').astype(np.float32) * 0.001

    # ── Inference ─────────────────────────────────────────────
    def run_inference(self):
        with self.lock:
            if self.rgb_image is None:
                return
            rgb   = self.rgb_image.copy()
            depth = self.depth_image.copy() if self.depth_image is not None else None

        h, w   = rgb.shape[:2]
        results = self.model(rgb, verbose=False, conf=CONF_THRESHOLD)[0]

        detections = []
        ma         = MarkerArray()
        marker_id  = 0

        for box in results.boxes:
            cls_id = int(box.cls[0])
            label  = self.model.names[cls_id]
            conf   = float(box.conf[0])

            if label not in BVI_CLASSES:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # Distance — median over small central patch of bounding box
            dist = MAX_DEPTH
            if depth is not None:
                roi   = depth[
                    max(0, cy - 15):min(h, cy + 15),
                    max(0, cx - 15):min(w, cx + 15)]
                valid = roi[(roi > MIN_DEPTH) & (roi < MAX_DEPTH)]
                if len(valid) > 0:
                    dist = float(np.median(valid))

            # Which of the 5 zones does the box centre fall in?
            zone_idx = min(int(cx / w * 5), 4)
            zone     = ZONE_NAMES[zone_idx]

            detections.append({
                'label':      label,
                'distance':   round(dist, 2),
                'zone':       zone,
                'zone_idx':   zone_idx,
                'confidence': round(conf, 2),
            })

            # RViz2 label marker at the obstacle
            m = Marker()
            m.header.frame_id    = 'camera_depth_optical_frame'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = 'yolo_detections'
            m.id                 = marker_id
            m.type               = Marker.TEXT_VIEW_FACING
            m.action             = Marker.ADD
            m.pose.position.x    = ZONE_X_OFFSETS[zone_idx]
            m.pose.position.y    = -0.55
            m.pose.position.z    = min(dist, 4.0)
            m.pose.orientation.w = 1.0
            m.scale.z            = 0.14
            m.color              = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            m.text               = f'{label}\n{dist:.1f}m ({int(conf*100)}%)'
            ma.markers.append(m)
            marker_id += 1

            self.get_logger().info(
                f'  [{label}] {dist:.2f}m in [{zone}] conf={conf:.2f}',
                throttle_duration_sec=1.0)

        # Delete any stale markers from last frame
        for i in range(marker_id, marker_id + 20):
            d = Marker()
            d.header.frame_id = 'camera_depth_optical_frame'
            d.header.stamp    = self.get_clock().now().to_msg()
            d.ns              = 'yolo_detections'
            d.id              = i
            d.action          = Marker.DELETE
            ma.markers.append(d)

        msg_out      = String()
        msg_out.data = json.dumps(detections)
        self.pub_detections.publish(msg_out)
        self.pub_markers.publish(ma)


def main():
    rclpy.init()
    rclpy.spin(YoloNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
