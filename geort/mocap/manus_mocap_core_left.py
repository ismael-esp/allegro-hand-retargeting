# manus_core_left.py
import math
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import zmq

# --- Canonical frame (same as right-hand!) -----------------
def hand_to_canonical(points_xyz):
    """
    Build a stable hand frame from keypoints (same math for both hands).
    0: wrist, 5: index MCP, 9: middle MCP, 13: ring MCP
    Returns 21x3 points in the hand (wrist) frame.
    """
    hand_point = np.asarray(points_xyz, dtype=np.float32)

    z_axis = hand_point[9] - hand_point[0]           # wrist -> middle MCP
    z_axis = z_axis / (np.linalg.norm(z_axis) + 1e-9)

    # y_axis_aux = hand_point[5] - hand_point[13]      # index MCP -> ring MCP
    y_axis_aux = hand_point[13] - hand_point[5]      # index MCP -> ring MCP
    y_axis_aux = y_axis_aux / (np.linalg.norm(y_axis_aux) + 1e-9)

    x_axis = np.cross(y_axis_aux, z_axis)
    x_axis = x_axis / (np.linalg.norm(x_axis) + 1e-9)

    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / (np.linalg.norm(y_axis) + 1e-9)

    R = np.stack([x_axis, y_axis, z_axis], axis=1)   # 3x3, right-handed
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R
    T[:3,  3] = hand_point[0]

    Tinv = np.linalg.inv(T)
    hp_h = np.concatenate([hand_point, np.ones((21, 1), dtype=np.float32)], axis=1)
    out = (hp_h @ Tinv.T)[:, :3]
    return out

# --- FK helper ------------------------------------------------
class ManusForwardKinematicsSolver:
    @staticmethod
    def make_T(pos, quat_xyzw):
        from scipy.spatial.transform import Rotation as R
        T = np.eye(4, dtype=np.float32)
        T[:3, 3] = pos
        T[:3, :3] = R.from_quat(quat_xyzw).as_matrix().astype(np.float32)
        return T

    def solve_keypoints(self, positions, orientations):
        # chains
        fingers = [
            [0,1,2,3,4],      # thumb
            [0,5,6,7,8],      # index
            [0,9,10,11,12],   # middle
            [0,13,14,15,16],  # ring
            [0,17,18,19,20],  # pinky
        ]
        kps = {}
        for chain in fingers:
            T_world = np.eye(4, dtype=np.float32)
            for idx in chain:
                T = self.make_T(np.array(positions[idx]),
                                np.array(orientations[idx]))
                T_world = T_world @ T
                if idx not in kps:
                    kps[idx] = T_world[:3, 3].copy()
        return kps

# --- ROS node: mirror X for LEFT to match right-hand model ---
class Manus(Node):
    def __init__(self):
        super().__init__("manus_left_visualizer")

        # anthropometric offsets used in FK
        self.pos = np.array([
            [0.0, 0.0, 0.0],
            [0.024950, 0.0000, 0.025320],
            [0.0000,   0.0000, 0.032742],
            [0.0000,   0.0000, 0.028739],
            [0.0000,   0.0000, 0.028739],

            [0.011181, 0.0000, 0.052904],
            [0.0000,   0.0000, 0.038257],
            [0.0000,   0.0000, 0.020884],
            [0.0000,   0.0000, 0.018759],

            [0.0000,   0.0000, 0.051287],
            [0.0000,   0.0000, 0.041861],
            [0.0000,   0.0000, 0.024766],
            [0.0000,   0.0000, 0.019683],

            [-0.011274, 0.0000, 0.049802],
            [0.0000,    0.0000, 0.039736],
            [0.0000,    0.0000, 0.023564],
            [0.0000,    0.0000, 0.019868],

            [-0.020145, 0.0000, 0.047309],
            [0.0000,    0.0000, 0.033175],
            [0.0000,    0.0000, 0.018020],
            [0.0000,    0.0000, 0.019129],
        ], dtype=np.float32)

        self.quat = None
        self.sub_x = self.create_subscription(Float32MultiArray, "/lh_x_manus_rotations", self._cb_x, 10)
        self.sub_y = self.create_subscription(Float32MultiArray, "/lh_y_manus_rotations", self._cb_y, 10)
        self.sub_z = self.create_subscription(Float32MultiArray, "/lh_z_manus_rotations", self._cb_z, 10)
        self.sub_q = self.create_subscription(Float32MultiArray, "/lh_manus_quats",      self._cb_q, 10)

        # ZMQ publisher
        #8765 # Right hand port
        self.port = 8766 # Left hand port
        ctx = zmq.Context()
        self.socket = ctx.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{self.port}")
        self.socket.setsockopt(zmq.SNDHWM, 0)

        self._x = []; self._y = []; self._z = []

    def _cb_x(self, msg): self._x = list(msg.data)
    def _cb_y(self, msg): self._y = list(msg.data)
    def _cb_z(self, msg): self._z = list(msg.data)
    def _cb_q(self, msg): self.quat = np.array(list(msg.data), dtype=np.float32).reshape(21,4)

    def run(self):
        fk = ManusForwardKinematicsSolver()
        count = 0
        while rclpy.ok():
            if self.quat is None: 
                rclpy.spin_once(self, timeout_sec=0.001)
                continue

            # FK -> canonical -> MIRROR X (left->right)
            keypoints = fk.solve_keypoints(self.pos, self.quat)
            keypoints = np.array([keypoints[i] for i in range(21)], dtype=np.float32)
            keypoints = hand_to_canonical(keypoints).astype(np.float32)
            keypoints[:, 0] *= -1.0  # <<< MIRROR across YZ plane

            self.socket.send(keypoints.tobytes())
            count += 1
            print("Broadcasting LEFT Manus Reading", count, keypoints.shape)

            rclpy.spin_once(self, timeout_sec=0.0)

def main(args=None):
    rclpy.init(args=args)
    node = Manus()
    execu = rclpy.executors.SingleThreadedExecutor()
    execu.add_node(node)
    t = threading.Thread(target=node.run, daemon=True)
    t.start()
    execu.spin()
    t.join()

if __name__ == "__main__":
    main()