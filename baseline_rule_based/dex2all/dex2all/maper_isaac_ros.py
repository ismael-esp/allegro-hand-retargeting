#!/usr/bin/env python3
import time, math
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def clamp(v, lo, hi): return max(lo, min(hi, v))

class IsaacToURGentle(Node):
    def __init__(self):
        super().__init__('isaac_to_ur_gentle_streamer')
        # ---- Params (tune here) ----
        self.declare_parameter('sim_js_topic', '/joint_states_re')   # Isaac joint states
        self.declare_parameter('ctrl_topic', '/scaled_joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('joints', [
            'shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
            'wrist_1_joint','wrist_2_joint','wrist_3_joint'
        ])
        self.declare_parameter('rate_hz', 400.0)     # 20–30 Hz
        self.declare_parameter('horizon_s', 0.45)   # 0.4–0.6 s
        self.declare_parameter('ema_alpha', 0.12)   # stronger smoothing
        self.declare_parameter('v_max', 1.2)        # rad/s clamp
        self.declare_parameter('a_max', 2.0)        # rad/s^2 clamp

        p = self.get_parameter
        self.sim_js = p('sim_js_topic').value
        self.ctrl   = p('ctrl_topic').value
        self.joints = list(p('joints').value)
        self.dt     = 1.0/float(p('rate_hz').value)
        self.H      = float(p('horizon_s').value)
        self.alpha  = float(p('ema_alpha').value)
        self.vmax   = float(p('v_max').value)
        self.amax   = float(p('a_max').value)

        # ---- ROS IO ----
        self.sub = self.create_subscription(JointState, self.sim_js, self._on_js, 50)
        self.pub = self.create_publisher(JointTrajectory, self.ctrl, 10)
        self.timer = self.create_timer(self.dt, self._tick)

        # ---- State ----
        self.pos_filt = None           # EMA-position
        self.target   = None           # latest filtered target (np.array)
        self.cmd      = None           # last commanded joint vector
        self.vel      = None           # smoothed command velocity
        self.get_logger().info(f"{self.sim_js} → {self.ctrl} | H={self.H:.2f}s @ {1/self.dt:.0f}Hz")

    def _on_js(self, msg: JointState):
        m = {n: p for n, p in zip(msg.name, msg.position)}
        if not all(j in m for j in self.joints):
            return
        cur = np.array([m[j] for j in self.joints], dtype=float)
        if self.pos_filt is None:
            self.pos_filt = cur
        else:
            self.pos_filt = self.alpha*cur + (1.0-self.alpha)*self.pos_filt
        self.target = self.pos_filt.copy()

    def _tick(self):
        if self.target is None:
            return

        # Initialize command & velocity trackers on first tick
        if self.cmd is None:
            self.cmd = self.target.copy()
            self.vel = np.zeros_like(self.cmd)

        # Compute limited velocity towards target
        #   1) raw desired vel = (target - cmd)/dt
        #   2) clamp |v| <= v_max
        #   3) accel-limit: clamp change in v, |dv| <= a_max*dt
        raw_v = (self.target - self.cmd) / self.dt
        v_clamped = np.clip(raw_v, -self.vmax, self.vmax)
        dv = v_clamped - self.vel
        a_step = self.amax * self.dt
        self.vel += np.clip(dv, -a_step, a_step)

        # Integrate one step
        self.cmd = self.cmd + self.vel * self.dt

        # Build positions-only trajectory with longer horizon
        jt = JointTrajectory()
        jt.joint_names = self.joints
        jt.header.stamp = self.get_clock().now().to_msg()

        p = JointTrajectoryPoint()
        p.positions = self.cmd.tolist()
        p.time_from_start = Duration(seconds=self.H).to_msg()

        jt.points = [p]
        self.pub.publish(jt)

def main():
    rclpy.init()
    rclpy.spin(IsaacToURGentle())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
