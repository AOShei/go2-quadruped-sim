#!/usr/bin/env python3
# Copyright (c) 2024-2025 Andrew O'Shei
#
# This software is released under the BSD-3-Clause License.
# See LICENSE file in the root directory for full license information.

"""
Body Pose Demo Script for Unitree Go2

This script demonstrates the dynamic body pose control capabilities of the
CHAMP-based quadruped controller by cycling through various body orientations
and heights. Perfect for showcasing the robot's stability and control flexibility.
"""

import rclpy
from rclpy.node import Node
from unitree_go2_msgs.msg import Pose
import math
import time


class BodyPoseDemo(Node):
    """
    Demonstrates body pose control by executing a series of poses
    that showcase the robot's stability and range of motion.
    """

    def __init__(self):
        super().__init__('body_pose_demo')

        # Publisher for body pose commands
        self.pose_publisher = self.create_publisher(Pose, '/body_pose', 10)

        # Wait for publisher to be ready
        time.sleep(0.5)

        self.get_logger().info('Body Pose Demo Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info('This demo will showcase various body pose capabilities')
        self.get_logger().info('='*60)

    def publish_pose(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        """Publish a body pose command."""
        msg = Pose()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.roll = float(roll)
        msg.pitch = float(pitch)
        msg.yaw = float(yaw)
        self.pose_publisher.publish(msg)

    def smooth_transition(self, start_pose, end_pose, duration, description=""):
        """Smoothly transition from start_pose to end_pose over duration."""
        if description:
            self.get_logger().info(f'\n{description}')

        steps = int(duration * 20)  # 20 steps per second
        start_time = time.time()

        for i in range(steps + 1):
            alpha = i / steps  # Interpolation factor (0 to 1)

            # Linear interpolation for each DOF
            x = start_pose[0] + alpha * (end_pose[0] - start_pose[0])
            y = start_pose[1] + alpha * (end_pose[1] - start_pose[1])
            z = start_pose[2] + alpha * (end_pose[2] - start_pose[2])
            roll = start_pose[3] + alpha * (end_pose[3] - start_pose[3])
            pitch = start_pose[4] + alpha * (end_pose[4] - start_pose[4])
            yaw = start_pose[5] + alpha * (end_pose[5] - start_pose[5])

            self.publish_pose(x, y, z, roll, pitch, yaw)
            time.sleep(1.0 / 20.0)  # 20 Hz

    def run_demo(self):
        """Execute the complete body pose demonstration sequence with smooth transitions."""

        # Define the sequence of poses - each will smoothly transition to the next
        pose_sequence = [
            # (x, y, z, roll, pitch, yaw, hold_duration, description)
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, '[1/10] Starting at neutral pose'),
            (0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 2.5, '[2/10] Raising body height (+6cm)'),
            (0.0, 0.0, -0.05, 0.0, 0.0, 0.0, 2.5, '[3/10] Lowering body height (-5cm)'),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, '[4/10] Returning to neutral'),
            (0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 2.5, '[5/10] Pitching forward (climbing pose, +0.2 rad)'),
            (0.0, 0.0, 0.0, 0.0, -0.2, 0.0, 2.5, '[6/10] Pitching backward (-0.2 rad)'),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, '[7/10] Returning to neutral'),
            (0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 2.5, '[8/10] Rolling left (+0.2 rad)'),
            (0.0, 0.0, 0.0, -0.2, 0.0, 0.0, 2.5, '[9/10] Rolling right (-0.2 rad)'),
            (0.0, 0.0, 0.03, 0.1, 0.15, 0.0, 2.5, '[10/10] Combined: raised height, forward pitch, left roll'),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 'Demo complete! Returning to neutral pose'),
        ]

        self.get_logger().info('\nStarting body pose demonstration with smooth transitions...\n')

        # Transition duration between poses (seconds)
        transition_duration = 2.0

        # Start from the first pose
        current_pose = pose_sequence[0][:6]

        for i, target_pose_data in enumerate(pose_sequence):
            target_pose = target_pose_data[:6]
            hold_duration = target_pose_data[6]
            description = target_pose_data[7]

            # Smoothly transition from current pose to target pose
            if i > 0:  # Skip transition for the first pose
                self.smooth_transition(
                    current_pose,
                    target_pose,
                    transition_duration,
                    description=f"Transitioning to: {description}"
                )
            else:
                # For the first pose, just log and hold
                self.get_logger().info(f'\n{description}')
                self.get_logger().info(f'  Position: x={target_pose[0]:.3f}, y={target_pose[1]:.3f}, z={target_pose[2]:.3f}')
                self.get_logger().info(f'  Orientation: roll={target_pose[3]:.3f}, pitch={target_pose[4]:.3f}, yaw={target_pose[5]:.3f}')

            # Hold at the target pose
            start_time = time.time()
            while (time.time() - start_time) < hold_duration:
                self.publish_pose(*target_pose)
                time.sleep(0.05)  # 20 Hz publishing

            # Update current pose for next iteration
            current_pose = target_pose

        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('Body Pose Demo Complete!')
        self.get_logger().info('='*60)
        self.get_logger().info('The robot is now in neutral pose.')
        self.get_logger().info('You can now experiment with your own body pose commands:')
        self.get_logger().info('  ros2 topic pub /body_pose unitree_go2_msgs/msg/Pose "{z: 0.05}"')
        self.get_logger().info('='*60 + '\n')


def main(args=None):
    rclpy.init(args=args)

    demo_node = BodyPoseDemo()

    try:
        demo_node.run_demo()
    except KeyboardInterrupt:
        demo_node.get_logger().info('Demo interrupted by user')
    finally:
        # Reset to neutral pose before exiting
        demo_node.publish_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
