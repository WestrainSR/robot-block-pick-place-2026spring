#!/usr/bin/env python3
import argparse
import math
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    import cv2
    import numpy as np
except Exception:  # pragma: no cover - robot image dependency.
    cv2 = None
    np = None


@dataclass
class TargetObservation:
    cx_ratio: float
    cy_ratio: float
    area_ratio: float
    angle_deg: float
    aspect: float
    score: float
    source: str
    stamp: float


def parse_args(args=None):
    parser = argparse.ArgumentParser(description='Fine-align the chassis to a rectangular placement target.')
    parser.add_argument('--rgb-topic', default='/ascamera/camera_publisher/rgb0/image')
    parser.add_argument('--cmd-vel-topic', default='/controller/cmd_vel')
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--rate', type=float, default=12.0)
    parser.add_argument('--stable-frames', type=int, default=5)
    parser.add_argument('--stale-seconds', type=float, default=0.35)
    parser.add_argument('--desired-cx', type=float, default=0.50)
    parser.add_argument('--desired-cy', type=float, default=0.60)
    parser.add_argument('--center-tolerance', type=float, default=0.025)
    parser.add_argument('--distance-mode', choices=['cy', 'area'], default='cy')
    parser.add_argument('--target-area-ratio', type=float, default=0.080)
    parser.add_argument('--area-tolerance-ratio', type=float, default=0.018)
    parser.add_argument('--target-angle-deg', type=float, default=0.0)
    parser.add_argument('--angle-tolerance-deg', type=float, default=10.0)
    parser.add_argument('--align-angle', action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument('--use-strafe', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--max-linear-x', type=float, default=0.055)
    parser.add_argument('--max-linear-y', type=float, default=0.040)
    parser.add_argument('--max-angular', type=float, default=0.22)
    parser.add_argument('--forward-k', type=float, default=0.28)
    parser.add_argument('--lateral-k', type=float, default=0.32)
    parser.add_argument('--angular-k', type=float, default=0.75)
    parser.add_argument('--forward-sign', type=float, default=-1.0)
    parser.add_argument('--lateral-sign', type=float, default=-1.0)
    parser.add_argument('--angular-sign', type=float, default=1.0)
    parser.add_argument('--search-angular-speed', type=float, default=0.12)
    parser.add_argument('--roi-y-min-ratio', type=float, default=0.20)
    parser.add_argument('--roi-y-max-ratio', type=float, default=1.00)
    parser.add_argument('--min-area-ratio', type=float, default=0.0010)
    parser.add_argument('--max-area-ratio', type=float, default=0.45)
    parser.add_argument('--min-aspect', type=float, default=0.45)
    parser.add_argument('--max-aspect', type=float, default=2.20)
    parser.add_argument('--min-saturation', type=int, default=38)
    parser.add_argument('--min-value', type=int, default=35)
    parser.add_argument('--dry-run', action='store_true')
    return parser.parse_args(args)


class PlaceTargetAlign(Node):
    def __init__(self, args) -> None:
        super().__init__('competition_place_target_align')
        if cv2 is None or np is None:
            raise RuntimeError('OpenCV/numpy are required for place target alignment')
        self.args = args
        self.latest_image: Optional[Image] = None
        self.latest_image_time = 0.0
        self.pub = self.create_publisher(Twist, args.cmd_vel_topic, 1)
        self.create_subscription(Image, args.rgb_topic, self.image_callback, qos_profile_sensor_data)

    def image_callback(self, msg: Image) -> None:
        self.latest_image = msg
        self.latest_image_time = time.monotonic()

    def run(self) -> bool:
        deadline = time.monotonic() + max(0.1, float(self.args.timeout))
        period = 1.0 / max(1.0, float(self.args.rate))
        stable = 0
        last_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            obs = self.latest_observation()
            now = time.monotonic()
            if obs is None:
                stable = 0
                if now - last_log > 0.8:
                    self.get_logger().info('searching placement target')
                    last_log = now
                self.publish_search(period)
                continue

            errors = self.alignment_errors(obs)
            if self.errors_aligned(errors):
                stable += 1
                self.publish_stop()
                if stable >= max(1, int(self.args.stable_frames)):
                    self.get_logger().info(
                        'placement target aligned: '
                        f'cx={obs.cx_ratio:.3f}, cy={obs.cy_ratio:.3f}, area={obs.area_ratio:.4f}, '
                        f'angle={obs.angle_deg:.1f}, source={obs.source}, stable={stable}'
                    )
                    return True
                time.sleep(period)
                continue

            stable = 0
            twist = self.compute_twist(errors)
            if now - last_log > 0.35:
                self.get_logger().info(
                    'place align: '
                    f'cx={obs.cx_ratio:.3f}, cy={obs.cy_ratio:.3f}, area={obs.area_ratio:.4f}, '
                    f'angle={obs.angle_deg:.1f}, aspect={obs.aspect:.2f}, source={obs.source}, '
                    f'err=({errors["cx"]:.3f},{errors["distance"]:.3f},{errors["angle_deg"]:.1f}deg), '
                    f'cmd=({twist.linear.x:.3f},{twist.linear.y:.3f},{twist.angular.z:.3f})'
                )
                last_log = now
            self.publish_for(twist, period)

        self.publish_stop(count=10)
        self.get_logger().error('placement target alignment timeout')
        return False

    def latest_observation(self) -> Optional[TargetObservation]:
        msg = self.latest_image
        if msg is None or time.monotonic() - self.latest_image_time > float(self.args.stale_seconds):
            return None
        frame = self.image_to_bgr(msg)
        if frame is None:
            return None
        return self.detect_target(frame)

    def image_to_bgr(self, msg: Image):
        encoding = str(msg.encoding or '').lower()
        channels_by_encoding = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1}
        channels = channels_by_encoding.get(encoding)
        if channels is None or msg.width <= 0 or msg.height <= 0:
            return None
        expected = int(msg.width) * channels
        if msg.step < expected:
            return None
        try:
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            rows = raw.reshape((int(msg.height), int(msg.step)))
            pixels = rows[:, :expected].reshape((int(msg.height), int(msg.width), channels))
            if encoding == 'bgr8':
                return pixels.copy()
            if encoding == 'rgb8':
                return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
            if encoding == 'bgra8':
                return cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
            if encoding == 'rgba8':
                return cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
            if encoding == 'mono8':
                return cv2.cvtColor(pixels[:, :, 0], cv2.COLOR_GRAY2BGR)
        except Exception:
            return None
        return None

    def detect_target(self, frame) -> Optional[TargetObservation]:
        height, width = frame.shape[:2]
        y1 = int(self.clamp(float(self.args.roi_y_min_ratio), 0.0, 1.0) * height)
        y2 = int(self.clamp(float(self.args.roi_y_max_ratio), 0.0, 1.0) * height)
        if y2 <= y1:
            y1, y2 = 0, height
        roi = frame[y1:y2, :]
        candidates = []
        for source, mask in self.target_masks(roi):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                obs = self.observation_from_contour(contour, source, width, height, y1)
                if obs is not None:
                    candidates.append(obs)
        if not candidates:
            return None
        desired_cx = float(self.args.desired_cx)
        desired_cy = float(self.args.desired_cy)
        return max(
            candidates,
            key=lambda obs: obs.score
            - 0.20 * abs(obs.cx_ratio - desired_cx) * width
            - 0.12 * abs(obs.cy_ratio - desired_cy) * height,
        )

    def target_masks(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        sat_mask = (
            (hsv[:, :, 1] >= int(self.args.min_saturation))
            & (hsv[:, :, 2] >= int(self.args.min_value))
        ).astype(np.uint8) * 255
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 45, 135)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        sat_mask = cv2.morphologyEx(sat_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        sat_mask = cv2.morphologyEx(sat_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        edge_mask = cv2.dilate(edges, kernel, iterations=1)
        edge_mask = cv2.morphologyEx(edge_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        combined = cv2.bitwise_or(sat_mask, edge_mask)
        yield 'color', sat_mask
        yield 'edge', edge_mask
        yield 'combined', combined

    def observation_from_contour(
        self,
        contour,
        source: str,
        image_width: int,
        image_height: int,
        roi_y_offset: int,
    ) -> Optional[TargetObservation]:
        area = float(cv2.contourArea(contour))
        area_ratio = area / max(1.0, float(image_width * image_height))
        if area_ratio < float(self.args.min_area_ratio) or area_ratio > float(self.args.max_area_ratio):
            return None
        rect = cv2.minAreaRect(contour)
        (cx, cy), (rw, rh), angle = rect
        rw = float(rw)
        rh = float(rh)
        if rw < 10.0 or rh < 10.0:
            return None
        aspect = max(rw, rh) / max(1e-6, min(rw, rh))
        if aspect < float(self.args.min_aspect) or aspect > float(self.args.max_aspect):
            return None
        x, y, bw, bh = cv2.boundingRect(contour)
        fill = area / max(1.0, float(bw * bh))
        if fill < 0.05:
            return None
        long_axis = float(angle)
        if rw < rh:
            long_axis += 90.0
        long_axis = self.normalize_axis_angle_deg(long_axis)
        squareness = 1.0 / max(1.0, aspect)
        score = area * (0.60 + min(fill, 1.0) + 0.35 * squareness)
        if source == 'combined':
            score *= 1.10
        return TargetObservation(
            cx_ratio=float(cx) / max(1.0, float(image_width)),
            cy_ratio=float(cy + roi_y_offset) / max(1.0, float(image_height)),
            area_ratio=area_ratio,
            angle_deg=long_axis,
            aspect=aspect,
            score=score,
            source=source,
            stamp=time.monotonic(),
        )

    def alignment_errors(self, obs: TargetObservation):
        cx_error = obs.cx_ratio - float(self.args.desired_cx)
        if self.args.distance_mode == 'area':
            distance_error = float(self.args.target_area_ratio) - obs.area_ratio
            distance_tolerance = float(self.args.area_tolerance_ratio)
        else:
            distance_error = obs.cy_ratio - float(self.args.desired_cy)
            distance_tolerance = float(self.args.center_tolerance)
        angle_error = self.normalize_axis_angle_deg(obs.angle_deg - float(self.args.target_angle_deg))
        return {
            'cx': cx_error,
            'distance': distance_error,
            'distance_tolerance': distance_tolerance,
            'angle_deg': angle_error,
        }

    def errors_aligned(self, errors) -> bool:
        center_ok = abs(errors['cx']) <= float(self.args.center_tolerance)
        distance_ok = abs(errors['distance']) <= float(errors['distance_tolerance'])
        angle_ok = (
            not bool(self.args.align_angle)
            or abs(errors['angle_deg']) <= float(self.args.angle_tolerance_deg)
        )
        return center_ok and distance_ok and angle_ok

    def compute_twist(self, errors) -> Twist:
        twist = Twist()
        if bool(self.args.use_strafe):
            twist.linear.y = self.clamp(
                float(self.args.lateral_sign) * float(self.args.lateral_k) * errors['cx'],
                -float(self.args.max_linear_y),
                float(self.args.max_linear_y),
            )
        else:
            twist.angular.z += self.clamp(
                -float(self.args.angular_k) * errors['cx'],
                -float(self.args.max_angular),
                float(self.args.max_angular),
            )
        twist.linear.x = self.clamp(
            float(self.args.forward_sign) * float(self.args.forward_k) * errors['distance'],
            -float(self.args.max_linear_x),
            float(self.args.max_linear_x),
        )
        if bool(self.args.align_angle):
            angle_rad = math.radians(errors['angle_deg'])
            twist.angular.z += self.clamp(
                float(self.args.angular_sign) * float(self.args.angular_k) * angle_rad,
                -float(self.args.max_angular),
                float(self.args.max_angular),
            )
        twist.angular.z = self.clamp(twist.angular.z, -float(self.args.max_angular), float(self.args.max_angular))
        return twist

    def publish_search(self, period: float) -> None:
        twist = Twist()
        twist.angular.z = float(self.args.search_angular_speed)
        self.publish_for(twist, min(period, 0.10))

    def publish_for(self, twist: Twist, duration: float) -> None:
        if self.args.dry_run:
            time.sleep(duration)
            return
        deadline = time.monotonic() + max(0.0, duration)
        while rclpy.ok() and time.monotonic() < deadline:
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)
        self.publish_stop(count=2)

    def publish_stop(self, count: int = 3) -> None:
        if self.args.dry_run:
            return
        zero = Twist()
        for _ in range(count):
            self.pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def normalize_axis_angle_deg(angle: float) -> float:
        while angle > 90.0:
            angle -= 180.0
        while angle <= -90.0:
            angle += 180.0
        return angle


def main(args=None) -> None:
    parsed = parse_args(args)
    rclpy.init()
    node = PlaceTargetAlign(parsed)
    try:
        ok = node.run()
    except KeyboardInterrupt:
        ok = False
    finally:
        if rclpy.ok():
            node.publish_stop(count=10)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if ok else 2)


if __name__ == '__main__':
    main()
