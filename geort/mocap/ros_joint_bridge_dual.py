#!/usr/bin/env python3
"""
Dual-hand bridge (ROS 2 Humble / Python 3.10):
- SUB from tcp://127.0.0.1:7777 (RIGHT) and tcp://127.0.0.1:7778 (LEFT)
- PUB to:
    RIGHT → /ra_/allegroHand_0/joint_cmd  (names: ra_joint_0_0 ... ra_joint_15_0)
    LEFT  → /la_/allegroHand_1/joint_cmd  (names: la_joint_0_0 ... la_joint_15_0)
"""

import argparse
import json
from typing import List

import zmq
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

RIGHT_TOPIC_DEFAULT = '/ra_/allegroHand_0/joint_cmd'
LEFT_TOPIC_DEFAULT  = '/la_/allegroHand_1/joint_cmd'

RIGHT_NAMES: List[str] = [
    "ra_joint_0_0",  "ra_joint_1_0",  "ra_joint_2_0",  "ra_joint_3_0",
    "ra_joint_4_0",  "ra_joint_5_0",  "ra_joint_6_0",  "ra_joint_7_0",
    "ra_joint_8_0",  "ra_joint_9_0",  "ra_joint_10_0", "ra_joint_11_0",
    "ra_joint_12_0", "ra_joint_13_0", "ra_joint_14_0", "ra_joint_15_0",
]
LEFT_NAMES: List[str] = [
    "la_joint_0_0",  "la_joint_1_0",  "la_joint_2_0",  "la_joint_3_0",
    "la_joint_4_0",  "la_joint_5_0",  "la_joint_6_0",  "la_joint_7_0",
    "la_joint_8_0",  "la_joint_9_0",  "la_joint_10_0", "la_joint_11_0",
    "la_joint_12_0", "la_joint_13_0", "la_joint_14_0", "la_joint_15_0",
]

class DualBridge(Node):
    def __init__(self,
                 right_addr: str,
                 left_addr: str,
                 right_topic: str,
                 left_topic: str,
                 hz: float,
                 clamp: bool,
                 lower: float,
                 upper: float):
        super().__init__('allegro_dual_joint_bridge')

        # Publishers
        self.pub_right = self.create_publisher(JointState, right_topic, 10)
        self.pub_left  = self.create_publisher(JointState, left_topic,  10)
        self.get_logger().info(f"Publishing RIGHT to: {right_topic}")
        self.get_logger().info(f"Publishing LEFT  to: {left_topic}")

        # ZMQ context
        self.ctx = zmq.Context.instance()

        # RIGHT SUB
        self.sub_r = self.ctx.socket(zmq.SUB)
        self.sub_r.connect(right_addr)
        self.sub_r.setsockopt(zmq.SUBSCRIBE, b"")
        self.get_logger().info(f"Subscribed RIGHT ZMQ: {right_addr}")

        # LEFT SUB
        self.sub_l = self.ctx.socket(zmq.SUB)
        self.sub_l.connect(left_addr)
        self.sub_l.setsockopt(zmq.SUBSCRIBE, b"")
        self.get_logger().info(f"Subscribed LEFT  ZMQ: {left_addr}")

        # Poller
        self.poller = zmq.Poller()
        self.poller.register(self.sub_r, zmq.POLLIN)
        self.poller.register(self.sub_l, zmq.POLLIN)

        # Timer (poll ZMQ and publish ROS)
        self.period = 1.0 / max(1e-6, hz)
        self.timer = self.create_timer(self.period, self._tick)

        self.clamp = clamp
        self.lower = lower
        self.upper = upper

    def destroy_node(self):
        try:
            self.poller.unregister(self.sub_r)
            self.poller.unregister(self.sub_l)
        except Exception:
            pass
        try:
            self.sub_r.close(0)
        except Exception:
            pass
        try:
            self.sub_l.close(0)
        except Exception:
            pass
        return super().destroy_node()

    def _publish_jointstate(self, names: List[str], qpos: List[float], right: bool):
        if self.clamp:
            qpos = [min(max(v, self.lower), self.upper) for v in qpos]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = qpos
        msg.velocity = []
        msg.effort = []
        if right:
            self.pub_right.publish(msg)
        else:
            self.pub_left.publish(msg)

    def _handle_message(self, s: str):
        """
        Accepts JSON string: {"hand": "right"|"left", "qpos":[16 floats], "stamp": float}
        """
        try:
            data = json.loads(s)
            qpos = data.get("qpos", None)
            hand = data.get("hand", None)

            if qpos is None or hand not in ("right", "left"):
                return

            if len(qpos) != 16:
                self.get_logger().warn(f"Bad qpos length {len(qpos)} for hand={hand}")
                return

            if hand == "right":
                self._publish_jointstate(RIGHT_NAMES, qpos, right=True)
            else:
                self._publish_jointstate(LEFT_NAMES, qpos, right=False)
        except Exception as e:
            self.get_logger().error(f"JSON decode/publish error: {e}")

    def _tick(self):
        # Non-blocking poll
        events = dict(self.poller.poll(timeout=0))
        if self.sub_r in events:
            try:
                s = self.sub_r.recv_string(flags=zmq.NOBLOCK)
                self._handle_message(s)
            except zmq.Again:
                pass
        if self.sub_l in events:
            try:
                s = self.sub_l.recv_string(flags=zmq.NOBLOCK)
                self._handle_message(s)
            except zmq.Again:
                pass

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--right_addr', type=str, default='tcp://127.0.0.1:7777',
                        help='ZMQ addr for RIGHT hand stream')
    parser.add_argument('--left_addr',  type=str, default='tcp://127.0.0.1:7778',
                        help='ZMQ addr for LEFT hand stream')
    parser.add_argument('--right_topic', type=str, default=RIGHT_TOPIC_DEFAULT,
                        help='ROS topic for RIGHT JointState')
    parser.add_argument('--left_topic',  type=str, default=LEFT_TOPIC_DEFAULT,
                        help='ROS topic for LEFT JointState')
    parser.add_argument('--hz', type=float, default=60.0, help='Publish cap (Hz)')
    parser.add_argument('--clamp', action='store_true', help='Clamp to [lower,upper]')
    parser.add_argument('--lower', type=float, default=-2.6, help='Clamp lower (rad)')
    parser.add_argument('--upper', type=float, default= 2.6, help='Clamp upper (rad)')
    args = parser.parse_args()

    rclpy.init()
    node = DualBridge(args.right_addr, args.left_addr, args.right_topic, args.left_topic,
                      args.hz, args.clamp, args.lower, args.upper)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
