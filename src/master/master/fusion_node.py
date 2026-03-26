#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math
from collections import deque

# ── Thresholds ───────────────────────────────────────────────
DANGER_DIST   = 0.8
WARNING_DIST  = 1.5
CLEAR_DIST    = 2.5

YAW_RATE_TURNING  = 10.0  # deg/s
PITCH_STAIR_UP    =  8.0  # deg
PITCH_STAIR_DOWN  = -8.0  # deg

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
    else:                  return ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.4)


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        self.sub_imu = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.sub_zones = self.create_subscription(
            Float32MultiArray, '/obstacles/zones', self.zones_callback, 10)

        self.pub_guidance = self.create_publisher(String,      '/navigation/guidance', 10)
        self.pub_markers  = self.create_publisher(MarkerArray, '/navigation/markers',  10)

        self.distances  = [4.0] * 5
	# ADD these two lines after: self.distances = [4.0] * 5
        self.zone_history = [deque(maxlen=3) for _ in range(5)]

        self.yaw_rate   = 0.0
        self.pitch      = 0.0
        self.roll       = 0.0
        self.prev_guidance = ''

        self.get_logger().info("Fusion node started")

    def imu_callback(self, msg):
        self.yaw_rate = math.degrees(msg.angular_velocity.z)
        q = msg.orientation
        sinp = max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x)))
        self.pitch = math.degrees(math.asin(sinp))
        sinr = 2*(q.w*q.x + q.y*q.z)
        cosr = 1 - 2*(q.x*q.x + q.y*q.y)
        self.roll  = math.degrees(math.atan2(sinr, cosr))

    def zones_callback(self, msg):
        if len(msg.data) != 5:
            return
    	# Feed each zone distance into its rolling window
        for i, d in enumerate(msg.data):
            self.zone_history[i].append(d)
    	# Use rolling average instead of raw value
        self.distances = [
            sum(h) / len(h) for h in self.zone_history
        ]
        self.fuse()


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
            if turning_left:
                guidance = 'STOP — turning into obstacle ahead-left'
                severity = 'DANGER'
            elif turning_right:
                guidance = 'STOP — turning into obstacle ahead-right'
                severity = 'DANGER'
            else:
                left_clear  = d[0] > WARNING_DIST and d[1] > WARNING_DIST
                right_clear = d[3] > WARNING_DIST and d[4] > WARNING_DIST
                if left_clear and not right_clear:
                    guidance = f'DANGER ahead {d[2]:.1f}m — move LEFT'
                elif right_clear and not left_clear:
                    guidance = f'DANGER ahead {d[2]:.1f}m — move RIGHT'
                elif left_clear and right_clear:
                    guidance = f'DANGER ahead {d[2]:.1f}m — move LEFT or RIGHT'
                else:
                    guidance = f'DANGER ahead {d[2]:.1f}m — STOP'
                severity = 'DANGER'
        elif c == 'WARN':
            if turning_left and l == 'DANGER':
                guidance = 'Turning into obstacle on left — slow down'
                severity = 'DANGER'
            elif turning_right and r == 'DANGER':
                guidance = 'Turning into obstacle on right — slow down'
                severity = 'DANGER'
            else:
                guidance = f'Caution — obstacle ahead {d[2]:.1f}m'
                severity = 'WARN'
        elif l == 'DANGER' or lf == 'DANGER':
            if turning_left:
                guidance = 'STOP — obstacle on left'
                severity = 'DANGER'
            else:
                guidance = f'Obstacle on left {min(d[0],d[1]):.1f}m'
                severity = 'WARN'
        elif r == 'DANGER' or rf == 'DANGER':
            if turning_right:
                guidance = 'STOP — obstacle on right'
                severity = 'DANGER'
            else:
                guidance = f'Obstacle on right {min(d[3],d[4]):.1f}m'
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
            c2 = RED if levels[i]=='DANGER' else YELLOW if levels[i]=='WARN' else GREEN
            dist_row += f'{c2}{ZONE_NAMES[i]}:{dist:.1f}m{RESET}  '
        print(BOLD + dist_row + RESET)
        print(BOLD + '═' * 55 + RESET)
        print(colour + BOLD + f'  ➤  {guidance}' + RESET)
        print(BOLD + '═' * 55 + RESET)

    def publish_markers(self, levels):
        ma = MarkerArray()
        for i, (dist, level) in enumerate(zip(self.distances, levels)):
            m = Marker()
            m.header.frame_id    = 'camera_depth_optical_frame'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns, m.id           = 'fusion_zones', i
            m.type, m.action     = Marker.CUBE, Marker.ADD
            m.pose.position.x    = ZONE_X_OFFSETS[i]
            m.pose.position.y    = 0.0
            m.pose.position.z    = min(dist, 4.0)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color              = zone_colour(level)
            ma.markers.append(m)

            t = Marker()
            t.header             = m.header
            t.ns, t.id           = 'fusion_labels', i + 10
            t.type               = Marker.TEXT_VIEW_FACING
            t.action             = Marker.ADD
            t.pose.position.x    = ZONE_X_OFFSETS[i]
            t.pose.position.y    = 0.18
            t.pose.position.z    = min(dist, 4.0)
            t.pose.orientation.w = 1.0
            t.scale.z            = 0.10
            t.color              = zone_colour(level)
            t.text               = f'{ZONE_NAMES[i]}\n{dist:.2f}m'
            ma.markers.append(t)
        self.pub_markers.publish(ma)


def main():
    rclpy.init()
    rclpy.spin(FusionNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
