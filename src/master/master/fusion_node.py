#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import json
from collections import deque

# ── Thresholds ───────────────────────────────────────────────
DANGER_DIST   = 0.8
WARNING_DIST  = 1.5
CLEAR_DIST    = 2.5

YAW_RATE_TURNING  = 10.0
PITCH_STAIR_UP    =  8.0
PITCH_STAIR_DOWN  = -8.0

ZONE_NAMES     = ['left_far', 'left', 'centre', 'right', 'right_far']
ZONE_X_OFFSETS = [-0.6, -0.3, 0.0, 0.3, 0.6]

RED    = '\033[91m'
YELLOW = '\033[93m'
GREEN  = '\033[92m'
RESET  = '\033[0m'
BOLD   = '\033[1m'


def classify(dist):
    if dist <= DANGER_DIST:    return 'DANGER'
    elif dist <= WARNING_DIST: return 'WARN'
    else:                      return 'CLEAR'

def zone_colour(level):
    if level == 'DANGER': return ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7)
    elif level == 'WARN':  return ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.7)
    else:                   return ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.4)


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        self.sub_imu = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.sub_zones = self.create_subscription(
            Float32MultiArray, '/obstacles/zones', self.zones_callback, 10)
        self.sub_detections = self.create_subscription(
            String, '/obstacles/detections', self.detections_callback, 10)

        self.pub_guidance = self.create_publisher(String,      '/navigation/guidance', 10)
        self.pub_markers  = self.create_publisher(MarkerArray, '/navigation/markers',  10)

        self.tf_br = tf2_ros.TransformBroadcaster(self)

        self.distances    = [4.0] * 5
        self.zone_history = [deque(maxlen=3) for _ in range(5)]
        self.yaw_rate     = 0.0
        self.yaw_angle    = 0.0
        self.pitch        = 0.0
        self.roll         = 0.0
        self.prev_guidance = ''

        # Latest YOLO detections — list of dicts keyed by zone_idx
        self.detections = []

        self.get_logger().info("Fusion node started")

    # ── IMU callback ─────────────────────────────────────────
    def imu_callback(self, msg):
        self.yaw_rate = math.degrees(msg.angular_velocity.z)
        q = msg.orientation

        sinp = max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x)))
        self.pitch = math.degrees(math.asin(sinp))

        sinr = 2*(q.w*q.x + q.y*q.z)
        cosr = 1 - 2*(q.x*q.x + q.y*q.y)
        self.roll = math.degrees(math.atan2(sinr, cosr))

        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self.yaw_angle = math.atan2(siny, cosy)

        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.yaw_angle / 2)
        t.transform.rotation.w = math.cos(self.yaw_angle / 2)
        self.tf_br.sendTransform(t)

    # ── Zones callback ────────────────────────────────────────
    def zones_callback(self, msg):
        if len(msg.data) != 5:
            return
        for i, d in enumerate(msg.data):
            self.zone_history[i].append(d)
        self.distances = [sum(h) / len(h) for h in self.zone_history]
        self.fuse()

    # ── Detections callback ───────────────────────────────────
    def detections_callback(self, msg):
        try:
            self.detections = json.loads(msg.data)
        except Exception:
            self.detections = []

    # ── Helper: get label for zone ────────────────────────────
    def get_label(self, zone_idx):
        """Return closest detection label in given zone, or empty string."""
        candidates = [
            d for d in self.detections if d.get('zone_idx') == zone_idx]
        if not candidates:
            return ''
        closest = min(candidates, key=lambda d: d['distance'])
        return closest['label']

    def label_str(self, zone_idx):
        """Format label for insertion into guidance string."""
        lbl = self.get_label(zone_idx)
        return f' ({lbl})' if lbl else ''

    # ── Fusion logic ──────────────────────────────────────────
    def fuse(self):
        d      = self.distances
        levels = [classify(x) for x in d]
        lf, l, c, r, rf = levels
        turning_left  = self.yaw_rate >  YAW_RATE_TURNING
        turning_right = self.yaw_rate < -YAW_RATE_TURNING

        guidance = ''
        severity = 'CLEAR'

        if self.pitch >= PITCH_STAIR_UP:
            guidance = 'RAMP UP — reduce speed'
            severity = 'WARN'
        elif self.pitch <= PITCH_STAIR_DOWN:
            guidance = 'RAMP DOWN / STEP — caution'
            severity = 'DANGER'
        elif c == 'DANGER':
            obj = self.label_str(2)   # centre zone
            if turning_left:
                guidance = f'STOP — turning into{obj} obstacle ahead-left'
                severity = 'DANGER'
            elif turning_right:
                guidance = f'STOP — turning into{obj} obstacle ahead-right'
                severity = 'DANGER'
            else:
                left_clear  = d[0] > WARNING_DIST and d[1] > WARNING_DIST
                right_clear = d[3] > WARNING_DIST and d[4] > WARNING_DIST
                if left_clear and not right_clear:
                    guidance = f'DANGER{obj} ahead {d[2]:.1f}m — move LEFT'
                elif right_clear and not left_clear:
                    guidance = f'DANGER{obj} ahead {d[2]:.1f}m — move RIGHT'
                elif left_clear and right_clear:
                    guidance = f'DANGER{obj} ahead {d[2]:.1f}m — move LEFT or RIGHT'
                else:
                    guidance = f'DANGER{obj} ahead {d[2]:.1f}m — STOP'
                severity = 'DANGER'
        elif c == 'WARN':
            obj = self.label_str(2)
            if turning_left and l == 'DANGER':
                guidance = f'Turning into{obj} obstacle on left — slow down'
                severity = 'DANGER'
            elif turning_right and r == 'DANGER':
                guidance = f'Turning into{obj} obstacle on right — slow down'
                severity = 'DANGER'
            else:
                guidance = f'Caution{obj} — obstacle ahead {d[2]:.1f}m'
                severity = 'WARN'
        elif l == 'DANGER' or lf == 'DANGER':
            zone_i = 1 if d[1] < d[0] else 0
            obj    = self.label_str(zone_i)
            if turning_left:
                guidance = f'STOP —{obj} obstacle on left'
                severity = 'DANGER'
            else:
                guidance = f'Obstacle{obj} on left {min(d[0],d[1]):.1f}m'
                severity = 'WARN'
        elif r == 'DANGER' or rf == 'DANGER':
            zone_i = 3 if d[3] < d[4] else 4
            obj    = self.label_str(zone_i)
            if turning_right:
                guidance = f'STOP —{obj} obstacle on right'
                severity = 'DANGER'
            else:
                guidance = f'Obstacle{obj} on right {min(d[3],d[4]):.1f}m'
                severity = 'WARN'
        else:
            if turning_left:
                guidance = 'Turning left — path clear'
            elif turning_right:
                guidance = 'Turning right — path clear'
            else:
                guidance = 'Path clear'
            severity = 'CLEAR'

        msg_out = String()
        msg_out.data = guidance
        self.pub_guidance.publish(msg_out)

        self.publish_markers(levels)

        if guidance == self.prev_guidance:
            return
        self.prev_guidance = guidance

        colour = RED if severity == 'DANGER' else YELLOW if severity == 'WARN' else GREEN
        print('\n' + BOLD + '═' * 55 + RESET)
        print(BOLD + ' IMU    │ ' + RESET +
              f'yaw_rate:{self.yaw_rate:+.1f}°/s  '
              f'pitch:{self.pitch:+.1f}°  '
              f'roll:{self.roll:+.1f}°')
        dist_row = ' ZONES  │ '
        for i, dist in enumerate(self.distances):
            c2 = RED if levels[i] == 'DANGER' else YELLOW if levels[i] == 'WARN' else GREEN
            dist_row += f'{c2}{ZONE_NAMES[i]}:{dist:.1f}m{RESET}  '
        print(BOLD + dist_row + RESET)
        print(BOLD + '═' * 55 + RESET)
        print(colour + BOLD + f'  ➤  {guidance}' + RESET)
        print(BOLD + '═' * 55 + RESET)

    # ── RViz2 markers ─────────────────────────────────────────
    def publish_markers(self, levels):
        ma    = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame = 'camera_depth_optical_frame'

        for i, (dist, level) in enumerate(zip(self.distances, levels)):
            col = zone_colour(level)

            corridor = Marker()
            corridor.header.frame_id    = frame
            corridor.header.stamp       = stamp
            corridor.ns                 = 'zone_corridors'
            corridor.id                 = i
            corridor.type               = Marker.CUBE
            corridor.action             = Marker.ADD
            corridor.pose.position.x    = ZONE_X_OFFSETS[i]
            corridor.pose.position.y    = 0.0
            corridor.pose.position.z    = min(dist, 4.0) / 2.0
            corridor.pose.orientation.w = 1.0
            corridor.scale.x            = 0.22
            corridor.scale.y            = 0.10
            corridor.scale.z            = min(dist, 4.0)
            corridor.color              = ColorRGBA(r=col.r, g=col.g, b=col.b, a=0.25)
            ma.markers.append(corridor)

            face = Marker()
            face.header.frame_id    = frame
            face.header.stamp       = stamp
            face.ns                 = 'obstacle_faces'
            face.id                 = i
            face.type               = Marker.CUBE
            face.action             = Marker.ADD
            face.pose.position.x    = ZONE_X_OFFSETS[i]
            face.pose.position.y    = 0.0
            face.pose.position.z    = min(dist, 4.0)
            face.pose.orientation.w = 1.0
            face.scale.x            = 0.22
            face.scale.y            = 0.40
            face.scale.z            = 0.05
            face.color              = col
            ma.markers.append(face)

            # Label — include YOLO object name if available
            lbl  = self.get_label(i)
            text = f'{lbl}\n{dist:.2f}m' if lbl else f'{ZONE_NAMES[i]}\n{dist:.2f}m'
            label = Marker()
            label.header.frame_id    = frame
            label.header.stamp       = stamp
            label.ns                 = 'zone_labels'
            label.id                 = i
            label.type               = Marker.TEXT_VIEW_FACING
            label.action             = Marker.ADD
            label.pose.position.x    = ZONE_X_OFFSETS[i]
            label.pose.position.y    = -0.30
            label.pose.position.z    = min(dist, 4.0)
            label.pose.orientation.w = 1.0
            label.scale.z            = 0.12
            label.color              = col
            label.text               = text
            ma.markers.append(label)

        # Guidance text
        guidance_marker = Marker()
        guidance_marker.header.frame_id    = frame
        guidance_marker.header.stamp       = stamp
        guidance_marker.ns                 = 'guidance_text'
        guidance_marker.id                 = 0
        guidance_marker.type               = Marker.TEXT_VIEW_FACING
        guidance_marker.action             = Marker.ADD
        guidance_marker.pose.position.x    = 0.0
        guidance_marker.pose.position.y    = -0.60
        guidance_marker.pose.position.z    = 1.5
        guidance_marker.pose.orientation.w = 1.0
        guidance_marker.scale.z            = 0.18
        worst_level = classify(self.distances[2])
        guidance_marker.color = zone_colour(worst_level)
        guidance_marker.text  = self.prev_guidance if self.prev_guidance else 'Initialising...'
        ma.markers.append(guidance_marker)

        # Direction arrow
        guidance_upper = self.prev_guidance.upper()
        show_arrow = False
        arrow_dir  = 0.0
        if 'MOVE LEFT' in guidance_upper or 'TURNING LEFT' in guidance_upper:
            show_arrow = True
            arrow_dir  = -1.0
        elif 'MOVE RIGHT' in guidance_upper or 'TURNING RIGHT' in guidance_upper:
            show_arrow = True
            arrow_dir  = 1.0

        arrow = Marker()
        arrow.header.frame_id    = frame
        arrow.header.stamp       = stamp
        arrow.ns                 = 'direction_arrow'
        arrow.id                 = 0
        arrow.type               = Marker.ARROW
        arrow.action             = Marker.ADD if show_arrow else Marker.DELETE
        arrow.pose.position.x    = 0.0
        arrow.pose.position.y    = 0.0
        arrow.pose.position.z    = 1.0
        angle = math.pi / 2 * arrow_dir
        arrow.pose.orientation.x = 0.0
        arrow.pose.orientation.y = math.sin(angle / 2)
        arrow.pose.orientation.z = 0.0
        arrow.pose.orientation.w = math.cos(angle / 2)
        arrow.scale.x            = 0.6
        arrow.scale.y            = 0.08
        arrow.scale.z            = 0.12
        arrow.color              = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        ma.markers.append(arrow)

        self.pub_markers.publish(ma)


def main():
    rclpy.init()
    rclpy.spin(FusionNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
