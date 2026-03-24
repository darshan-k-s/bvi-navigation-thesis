#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

ZONE_NAMES   = ['left_far', 'left', 'centre', 'right', 'right_far']
DANGER_DIST  = 0.8
WARNING_DIST = 1.5

RED    = '\033[91m'
YELLOW = '\033[93m'
GREEN  = '\033[92m'
RESET  = '\033[0m'
BOLD   = '\033[1m'

# X offsets for 5 zones spread across camera FOV
ZONE_X_OFFSETS = [-0.6, -0.3, 0.0, 0.3, 0.6]


def classify(dist):
    if dist <= DANGER_DIST:
        return 'DANGER',  RED,    '!!'
    elif dist <= WARNING_DIST:
        return 'WARN',    YELLOW, ' !'
    else:
        return 'CLEAR',   GREEN,  '  '


def make_colour(level):
    if level == 'DANGER':
        return ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7)
    elif level == 'WARN':
        return ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.7)
    else:
        return ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.4)


class FeedbackNode(Node):
    def __init__(self):
        super().__init__('feedback_node')
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/obstacles/zones',
            self.zones_callback, 10)

        self.pub_markers = self.create_publisher(
            MarkerArray, '/obstacles/markers', 10)

        self.get_logger().info("Feedback node started")
        self.prev_alerts = [None] * 5

    def zones_callback(self, msg):
        distances = list(msg.data)
        if len(distances) != 5:
            return

        alerts      = []
        changed     = False
        has_danger  = False
        has_warning = False

        for i, dist in enumerate(distances):
            level, colour, marker = classify(dist)
            alerts.append((level, colour, marker, dist))
            if level != self.prev_alerts[i]:
                changed = True
            if level == 'DANGER':  has_danger  = True
            if level == 'WARN':    has_warning = True

        # ── Publish RViz2 markers (always, not just on change) ──
        marker_array = MarkerArray()
        for i, (level, colour, marker, dist) in enumerate(alerts):
            m = Marker()
            m.header.frame_id    = 'camera_depth_optical_frame'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = 'obstacle_zones'
            m.id                 = i
            m.type               = Marker.CUBE
            m.action             = Marker.ADD
            m.pose.position.x    = ZONE_X_OFFSETS[i]
            m.pose.position.y    = 0.0
            m.pose.position.z    = min(dist, 4.0)  # depth = distance
            m.pose.orientation.w = 1.0
            m.scale.x            = 0.25
            m.scale.y            = 0.25
            m.scale.z            = 0.25
            m.color              = make_colour(level)
            marker_array.markers.append(m)

            # Zone label
            t = Marker()
            t.header.frame_id    = 'camera_depth_optical_frame'
            t.header.stamp       = m.header.stamp
            t.ns                 = 'zone_labels'
            t.id                 = i + 10
            t.type               = Marker.TEXT_VIEW_FACING
            t.action             = Marker.ADD
            t.pose.position.x    = ZONE_X_OFFSETS[i]
            t.pose.position.y    = 0.15
            t.pose.position.z    = min(dist, 4.0)
            t.pose.orientation.w = 1.0
            t.scale.z            = 0.12
            t.color              = make_colour(level)
            t.text               = f'{ZONE_NAMES[i]}\n{dist:.2f}m'
            marker_array.markers.append(t)

        self.pub_markers.publish(marker_array)

        # ── Terminal output (only on change) ──
        if not changed:
            return
        self.prev_alerts = [a[0] for a in alerts]

        print('\n' + BOLD + '═' * 55 + RESET)
        bar = ''
        for i, (level, colour, marker, dist) in enumerate(alerts):
            bar += f'{colour}{marker} {ZONE_NAMES[i]:^9}{RESET} │ '
        print(BOLD + ' ZONES  │ ' + RESET + bar)

        dist_row = ''
        for level, colour, marker, dist in alerts:
            dist_row += f'{colour}{dist:>6.2f}m   {RESET}   │ '
        print(BOLD + ' DIST   │ ' + RESET + dist_row)

        status_row = ''
        for level, colour, marker, dist in alerts:
            status_row += f'{colour}{level:^9}{RESET} │ '
        print(BOLD + ' STATUS │ ' + RESET + status_row)
        print('═' * 55)

        if has_danger:
            danger_zones = [ZONE_NAMES[i] for i, a in enumerate(alerts) if a[0] == 'DANGER']
            print(RED + BOLD + f'  ⚠  DANGER — obstacle within {DANGER_DIST}m [{", ".join(danger_zones)}]' + RESET)
        elif has_warning:
            warn_zones = [ZONE_NAMES[i] for i, a in enumerate(alerts) if a[0] == 'WARN']
            print(YELLOW + BOLD + f'  ⚡  WARNING — obstacle within {WARNING_DIST}m [{", ".join(warn_zones)}]' + RESET)
        else:
            print(GREEN + BOLD + '  ✓  PATH CLEAR' + RESET)
        print('═' * 55)


def main():
    rclpy.init()
    rclpy.spin(FeedbackNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
