#!/usr/bin/env python3
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class DexToAllegroMapper(Node):
    def __init__(self):
        super().__init__('la_dex_to_allegro_mapper')

        # ── I/O topics ─────────────────────────────────────────
        self.sub = self.create_subscription(JointState, '/la_dex_joint_states', self.joint_callback, 20)
        self.pub = self.create_publisher(JointState, '/la_/allegroHand_1/joint_cmd', 20)

        # Allegro joint order (16-DoF)
        self.allegro_names = [
            'la_joint_0_0','la_joint_1_0','la_joint_2_0','la_joint_3_0',
            'la_joint_4_0','la_joint_5_0','la_joint_6_0','la_joint_7_0',
            'la_joint_8_0','la_joint_9_0','la_joint_10_0','la_joint_11_0',
            'la_joint_12_0','la_joint_13_0','la_joint_14_0','la_joint_15_0'
        ]

        # dex_idx -> allegro_idx (from your code)
        self.dex2alg = {
            3:0, 7:1, 8:2, 9:3,    # index
            4:4,10:5,11:6,12:7,    # middle
            5:8,13:9,14:10,15:11,  # ring
            20:12,21:13,22:14,23:15  # thumb
        }

        # ── Parameters (tune at runtime) ───────────────────────
        # Effort scaling
        self.declare_parameter('effort_min', 0.20)      # baseline squeeze
        self.declare_parameter('effort_max', 0.90)      # cap (keep <1 unless you know HW limits)
        self.declare_parameter('effort_gamma', 1.2)     # nonlinearity: >1 = more boost near closed
        self.declare_parameter('yaw_effort_scale', 0.35)  # yaw joints less torque
        self.declare_parameter('thumb_effort_scale', 1.15) # thumb a bit stronger
        self.declare_parameter('use_velocity_ff', False)
        self.declare_parameter('vel_ff_gain', 0.0)      # set >0 only if your driver uses velocity[]
        self.declare_parameter('vel_sat', 2.0)          # rad/s clamp on feed-forward

        # Open/close calibration (rough defaults; override via params if needed)
        # Use your proven “open” preset as the baseline so c ∈ [0..1] makes sense.
        open_default = [  # from your OPEN preset for left hand
            0.0,-0.15,-0.15,-0.15,  0.0,-0.15,-0.15,-0.15,
            0.0,-0.15,-0.15,-0.15,  0.50,-0.10,-0.10,-0.10
        ]
        close_default = [  # from your “Totally close” preset (approx)
            0.0,1.60,1.60,1.20,  0.0,1.60,1.60,1.20,
            0.0,1.60,1.60,1.20,  1.20,1.10,1.30,1.20
        ]
        self.declare_parameter('open_pose', open_default)
        self.declare_parameter('close_pose', close_default)

        # Index sets for scaling
        self.yaw_idxs   = {0,4,8}         # index/middle/ring yaw
        self.thumb_idxs = {12,13,14,15}   # thumb joints

        # State for velocity feed-forward
        self._last_pos = None
        self._last_t   = None

        self.get_logger().info("Dex→Allegro mapper ready (positions + effort).")

    # Helper to read param inside callback
    def P(self, name):
        return self.get_parameter(name).value

    def joint_callback(self, msg: JointState):
        # Map dex positions → Allegro (16)
        pos = [0.0]*16
        for dex_idx, alg_idx in self.dex2alg.items():
            if dex_idx < len(msg.position):
                pos[alg_idx] = float(msg.position[dex_idx])

        # ---- Velocity feed-forward (optional) ----
        vel = [0.0]*16
        now = time.time()
        if self.P('use_velocity_ff') and (self._last_pos is not None) and (self._last_t is not None):
            dt = max(1e-3, now - self._last_t)
            vel_raw = (np.array(pos) - np.array(self._last_pos)) / dt
            vel = np.clip(vel_raw, -self.P('vel_sat'), self.P('vel_sat')) * self.P('vel_ff_gain')
            vel = vel.tolist()
        self._last_pos = pos[:]
        self._last_t   = now

        # ---- Effort (grip strength) based on closure fraction ----
        open_pose  = list(self.P('open_pose'))
        close_pose = list(self.P('close_pose'))
        # protect size
        if len(open_pose) != 16 or len(close_pose) != 16:
            open_pose  = [0.0]*16
            close_pose = [1.0]*16

        # closure fraction per joint: c=0 (open) .. 1 (closed)
        c = []
        for i in range(16):
            o = open_pose[i]; cl = close_pose[i]
            span = max(1e-3, cl - o)
            ci = (pos[i] - o) / span
            c.append(float(np.clip(ci, 0.0, 1.0)))

        c = np.array(c)
        # nonlinearity and scaling
        eff_min   = float(self.P('effort_min'))
        eff_max   = float(self.P('effort_max'))
        gamma     = float(self.P('effort_gamma'))
        yaw_scale = float(self.P('yaw_effort_scale'))
        th_scale  = float(self.P('thumb_effort_scale'))

        effort = eff_min + (eff_max - eff_min) * (c ** gamma)

        # scale yaw lower (0,4,8) and thumb slightly higher (12..15)
        for i in range(16):
            if i in self.yaw_idxs:
                effort[i] *= yaw_scale
            elif i in self.thumb_idxs:
                effort[i] *= th_scale

        effort = np.clip(effort, 0.0, 1.5).tolist()  # final clamp (don’t exceed driver’s expected range)

        # ---- Publish command ----
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = self.allegro_names
        out.position = pos
        out.velocity = vel               # many drivers ignore; harmless if unused
        out.effort   = effort            # << the important bit for stronger grasp
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = DexToAllegroMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()