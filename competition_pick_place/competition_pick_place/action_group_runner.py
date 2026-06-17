#!/usr/bin/env python3
import argparse
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

try:
    from servo_controller.action_group_controller import ActionGroupController
    from servo_controller_msgs.msg import ServosPosition
except Exception:  # pragma: no cover - only available on the robot image.
    ActionGroupController = None
    ServosPosition = None


def parse_args(args=None):
    parser = argparse.ArgumentParser(description='Run one robot arm action group.')
    parser.add_argument('--action', default='navigation_place')
    parser.add_argument('--action-group-path', default='/home/ubuntu/software/arm_pc/ActionGroups')
    parser.add_argument('--controller-ready-service', default='/controller_manager/init_finish')
    parser.add_argument('--ready-timeout', type=float, default=8.0)
    parser.add_argument('--settle-seconds', type=float, default=0.3)
    parser.add_argument('--dry-run', action='store_true')
    return parser.parse_args(args)


def main(args=None) -> None:
    parsed = parse_args(args)
    if parsed.dry_run:
        print(f'dry-run action group: {parsed.action}')
        return
    if ActionGroupController is None or ServosPosition is None:
        raise RuntimeError('servo_controller libraries are not available')

    rclpy.init()
    node = Node('competition_action_group_runner')
    try:
        pub = node.create_publisher(ServosPosition, 'servo_controller', 1)
        client = node.create_client(Trigger, parsed.controller_ready_service)
        if not client.wait_for_service(timeout_sec=max(0.1, parsed.ready_timeout)):
            raise RuntimeError(f'{parsed.controller_ready_service} service is not available')
        for _ in range(8):
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        node.get_logger().info(f'run action group {parsed.action}')
        ActionGroupController(pub, parsed.action_group_path).run_action(parsed.action)
        time.sleep(max(0.0, parsed.settle_seconds))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
