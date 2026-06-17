#!/usr/bin/env python3
import argparse
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def parse_args(args=None):
    parser = argparse.ArgumentParser(description='Drive a hard-coded open-loop path from one map pose to another.')
    parser.add_argument('--cmd-vel-topic', default='/controller/cmd_vel')
    parser.add_argument('--start-x', type=float, required=True)
    parser.add_argument('--start-y', type=float, required=True)
    parser.add_argument('--start-yaw', type=float, required=True)
    parser.add_argument('--end-x', type=float, required=True)
    parser.add_argument('--end-y', type=float, required=True)
    parser.add_argument('--end-yaw', type=float)
    parser.add_argument('--drive-mode', choices=['face_target', 'strafe'], default='face_target')
    parser.add_argument('--linear-speed', type=float, default=0.10)
    parser.add_argument('--angular-speed', type=float, default=0.35)
    parser.add_argument('--distance-scale', type=float, default=1.0)
    parser.add_argument('--angle-scale', type=float, default=1.0)
    parser.add_argument('--start-delay', type=float, default=2.0)
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument('--max-segment-seconds', type=float, default=20.0)
    parser.add_argument('--dry-run', action='store_true')
    return parser.parse_args(args)


class OpenLoopDrive(Node):
    def __init__(self, args) -> None:
        super().__init__('competition_open_loop_drive')
        self.args = args
        self.pub = self.create_publisher(Twist, args.cmd_vel_topic, 1)
        self.period = 1.0 / max(1.0, float(args.rate))

    def run(self) -> None:
        dx = self.args.end_x - self.args.start_x
        dy = self.args.end_y - self.args.start_y
        distance = math.hypot(dx, dy) * max(0.0, self.args.distance_scale)
        travel_heading = math.atan2(dy, dx) if distance > 1e-6 else self.args.start_yaw
        first_turn = 0.0
        if self.args.drive_mode == 'face_target':
            first_turn = normalize_angle(travel_heading - self.args.start_yaw) * self.args.angle_scale
        final_turn = self.final_turn(travel_heading)

        self.get_logger().info(
            'open-loop plan: '
            f'mode={self.args.drive_mode}, first_turn={first_turn:.3f}rad, distance={distance:.3f}m, '
            f'final_turn={final_turn if final_turn is None else round(final_turn, 3)}rad'
        )
        self.validate_plan(first_turn, distance, final_turn)
        if self.args.dry_run:
            return

        if self.args.start_delay > 0.0:
            self.get_logger().info(f'starting in {self.args.start_delay:.1f}s')
            time.sleep(self.args.start_delay)

        self.publish_stop()
        if self.args.drive_mode == 'face_target':
            self.rotate(first_turn, 'initial turn')
            self.drive_forward(distance, 'straight segment')
        else:
            self.drive_strafe(dx, dy, distance, 'straight strafe segment')
        if final_turn is not None:
            self.rotate(final_turn, 'final turn')
        self.publish_stop(count=10)
        self.get_logger().info('open-loop drive complete')

    def validate_plan(self, first_turn: float, distance: float, final_turn: Optional[float]) -> None:
        max_segment = max(0.1, float(self.args.max_segment_seconds))
        linear_speed = min(abs(float(self.args.linear_speed)), 0.60)
        angular_speed = min(abs(float(self.args.angular_speed)), 0.8)
        durations = []
        if angular_speed > 1e-6:
            durations.append(('initial turn', abs(first_turn) / angular_speed))
            if final_turn is not None:
                durations.append(('final turn', abs(final_turn) / angular_speed))
        if linear_speed > 1e-6:
            durations.append(('straight segment', distance / linear_speed))
        too_long = [(name, duration) for name, duration in durations if duration > max_segment]
        if too_long:
            detail = ', '.join(f'{name}={duration:.1f}s' for name, duration in too_long)
            raise RuntimeError(f'open-loop segment exceeds --max-segment-seconds={max_segment:.1f}: {detail}')

    def final_turn(self, travel_heading: float) -> Optional[float]:
        if self.args.end_yaw is None:
            return None
        if self.args.drive_mode == 'strafe':
            return normalize_angle(self.args.end_yaw - self.args.start_yaw) * self.args.angle_scale
        return normalize_angle(self.args.end_yaw - travel_heading) * self.args.angle_scale

    def rotate(self, angle: float, label: str) -> None:
        speed = min(abs(float(self.args.angular_speed)), 0.8)
        if speed <= 1e-6 or abs(angle) <= 1e-4:
            self.get_logger().info(f'{label}: skipped')
            return
        duration = abs(angle) / speed
        twist = Twist()
        twist.angular.z = math.copysign(speed, angle)
        self.publish_for(twist, duration, label)

    def drive_forward(self, distance: float, label: str) -> None:
        speed = min(abs(float(self.args.linear_speed)), 0.60)
        if speed <= 1e-6 or distance <= 1e-4:
            self.get_logger().info(f'{label}: skipped')
            return
        duration = distance / speed
        twist = Twist()
        twist.linear.x = speed
        self.publish_for(twist, duration, label)

    def drive_strafe(self, dx: float, dy: float, distance: float, label: str) -> None:
        speed = min(abs(float(self.args.linear_speed)), 0.60)
        if speed <= 1e-6 or distance <= 1e-4:
            self.get_logger().info(f'{label}: skipped')
            return
        scaled_dx = dx * max(0.0, self.args.distance_scale)
        scaled_dy = dy * max(0.0, self.args.distance_scale)
        cos_yaw = math.cos(self.args.start_yaw)
        sin_yaw = math.sin(self.args.start_yaw)
        body_x = cos_yaw * scaled_dx + sin_yaw * scaled_dy
        body_y = -sin_yaw * scaled_dx + cos_yaw * scaled_dy
        duration = distance / speed
        twist = Twist()
        twist.linear.x = body_x / duration
        twist.linear.y = body_y / duration
        self.publish_for(twist, duration, label)

    def publish_for(self, twist: Twist, duration: float, label: str) -> None:
        self.get_logger().info(
            f'{label}: duration={duration:.2f}s, '
            f'linear.x={twist.linear.x:.3f}, linear.y={twist.linear.y:.3f}, angular.z={twist.angular.z:.3f}'
        )
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.period)
        self.publish_stop()
        time.sleep(0.2)

    def publish_stop(self, count: int = 5) -> None:
        zero = Twist()
        for _ in range(count):
            self.pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.03)


def main(args=None) -> None:
    parsed = parse_args(args)
    rclpy.init()
    node = OpenLoopDrive(parsed)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop(count=10)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
