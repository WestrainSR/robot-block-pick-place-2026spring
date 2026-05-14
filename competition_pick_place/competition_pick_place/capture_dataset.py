#!/usr/bin/env python3
import csv
import os
import re
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


VALID_CLASSES = {'redstone', 'glass', 'glowstone', 'grass', 'mixed', 'background'}


def safe_name(value: str) -> str:
    value = value.strip().lower()
    value = re.sub(r'[^a-z0-9_-]+', '_', value)
    return value.strip('_') or 'unlabeled'


def safe_optional_name(value: str) -> str:
    value = value.strip()
    if not value:
        return ''
    return safe_name(value)


class RobotImageCapture(Node):
    def __init__(self) -> None:
        super().__init__('block_dataset_capture')

        self.declare_parameter('camera_topic', '/ascamera/camera_publisher/rgb0/image')
        self.declare_parameter('output_dir', '/home/ubuntu/datasets/block_dataset/raw')
        self.declare_parameter('class_name', 'mixed')
        self.declare_parameter('scene_name', '')
        self.declare_parameter('count', 120)
        self.declare_parameter('interval', 0.25)
        self.declare_parameter('start_delay', 5.0)
        self.declare_parameter('image_format', 'jpg')
        self.declare_parameter('jpeg_quality', 95)
        self.declare_parameter('min_free_mb', 300)

        self.camera_topic = str(self.get_parameter('camera_topic').value)
        self.output_dir = Path(str(self.get_parameter('output_dir').value))
        self.class_name = safe_name(str(self.get_parameter('class_name').value))
        self.scene_name = safe_optional_name(str(self.get_parameter('scene_name').value))
        self.count = int(self.get_parameter('count').value)
        self.interval = float(self.get_parameter('interval').value)
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.image_format = safe_name(str(self.get_parameter('image_format').value))
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.min_free_mb = int(self.get_parameter('min_free_mb').value)

        if self.class_name not in VALID_CLASSES:
            valid = ', '.join(sorted(VALID_CLASSES))
            raise RuntimeError(f'class_name must be one of: {valid}')
        if self.count <= 0:
            raise RuntimeError('count must be positive')
        if self.interval <= 0:
            raise RuntimeError('interval must be positive')
        if self.image_format not in {'jpg', 'jpeg', 'png'}:
            raise RuntimeError('image_format must be jpg/jpeg/png')

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        session_name = f'{self.scene_name}_{timestamp}' if self.scene_name else timestamp
        session_parts = [self.class_name, session_name]
        self.session_dir = self.output_dir.joinpath(*session_parts)
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.manifest_path = self.session_dir / 'manifest.csv'

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_stamp: Optional[float] = None
        self.last_saved_stamp: Optional[float] = None
        self.saved = 0
        self.started_at = time.monotonic()
        self.next_capture_at = self.started_at + self.start_delay
        self.manifest = self.manifest_path.open('w', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(
            self.manifest,
            fieldnames=['index', 'file', 'class_name', 'scene_name', 'ros_stamp', 'saved_at'],
        )
        self.writer.writeheader()

        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            5,
        )
        self.timer = self.create_timer(0.05, self.capture_tick)

        self.get_logger().info(f'camera topic: {self.camera_topic}')
        self.get_logger().info(f'saving {self.count} images to: {self.session_dir}')
        self.get_logger().info(f'class={self.class_name}, scene={self.scene_name or "-"}, start_delay={self.start_delay}s')

    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_frame = frame
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest_stamp = stamp or time.monotonic()

    def capture_tick(self) -> None:
        now = time.monotonic()
        if now < self.next_capture_at:
            return
        if self.saved >= self.count:
            self.finish()
            return
        if self.latest_frame is None:
            self.get_logger().warn('waiting for first camera frame...')
            self.next_capture_at = now + 1.0
            return
        if self.latest_stamp == self.last_saved_stamp:
            return
        self.ensure_free_space()

        self.saved += 1
        filename = f'{self.class_name}_{self.scene_name + "_" if self.scene_name else ""}{self.saved:05d}.{self.image_format}'
        path = self.session_dir / filename
        params = []
        if self.image_format in {'jpg', 'jpeg'}:
            params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        ok = cv2.imwrite(str(path), self.latest_frame, params)
        if not ok:
            raise RuntimeError(f'failed to save image: {path}')
        self.last_saved_stamp = self.latest_stamp
        self.writer.writerow({
            'index': self.saved,
            'file': filename,
            'class_name': self.class_name,
            'scene_name': self.scene_name,
            'ros_stamp': f'{self.latest_stamp:.6f}' if self.latest_stamp is not None else '',
            'saved_at': datetime.now().isoformat(timespec='seconds'),
        })
        self.manifest.flush()

        if self.saved == 1 or self.saved % 10 == 0 or self.saved == self.count:
            self.get_logger().info(f'captured {self.saved}/{self.count}: {path.name}')
        self.next_capture_at = now + self.interval

    def ensure_free_space(self) -> None:
        stat = os.statvfs(str(self.session_dir))
        free_mb = stat.f_bavail * stat.f_frsize / 1024 / 1024
        if free_mb < self.min_free_mb:
            raise RuntimeError(f'free disk space is too low: {free_mb:.1f} MB < {self.min_free_mb} MB')

    def finish(self) -> None:
        self.get_logger().info(f'capture finished: {self.saved} images saved in {self.session_dir}')
        self.manifest.close()
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotImageCapture()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if not node.manifest.closed:
                node.manifest.close()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
