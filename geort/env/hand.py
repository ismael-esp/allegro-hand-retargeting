# geort/env/hand.py
# Copyright (c) Meta Platforms
# All rights reserved.

import numpy as np
import sapien.core as sapien
from sapien.utils import Viewer

from geort.utils.config_utils import get_config, save_json
from geort.utils.hand_utils import (
    get_entity_by_name,
    get_active_joints,
    get_active_joint_indices,
)

import math


class HandKinematicModel:
    def __init__(
        self,
        scene=None,
        render=False,
        hand=None,
        hand_urdf="",
        n_hand_dof=16,
        base_link="base_link",
        joint_names=None,
        # PD controller (visualization)
        kp=400.0,
        kd=10.0,
        # NEW: retargeting corrections
        joint_sign=None,     # array-like, +1/-1 per joint (user order)
        joint_offset=None,   # array-like, radians per joint (user order)
        clamp_eps=1e-3,
    ):
        """
        joint_sign / joint_offset are applied in USER order *before* clipping/limits
        and before mapping to simulation order. This is crucial for LEFT-hand fixes.
        """
        if joint_names is None:
            joint_names = []

        self.engine = None
        if scene is None:
            engine = sapien.Engine()

            if render:
                renderer = sapien.VulkanRenderer()
                engine.set_renderer(renderer)
                print("Enable Render Mode.")
            else:
                renderer = None
            scene_config = sapien.SceneConfig()
            scene_config.default_dynamic_friction = 1.0
            scene_config.default_static_friction = 1.0
            scene_config.default_restitution = 0.00
            scene_config.contact_offset = 0.02
            scene_config.enable_pcm = False
            scene_config.solver_iterations = 25
            scene_config.solver_velocity_iterations = 1
            scene = engine.create_scene(scene_config)
            self.engine = engine

        self.scene = scene
        self.renderer = renderer
        self.clamp_eps = float(clamp_eps)

        if hand is not None:
            self.hand = hand
        else:
            loader = scene.create_urdf_loader()
            self.hand = loader.load(hand_urdf)
            # Upright palm default
            self.hand.set_root_pose(sapien.Pose([0, 0, 0.35], [0.695, 0, -0.718, 0]))

        self.pmodel = self.hand.create_pinocchio_model()

        # Base link
        self.base_link = get_entity_by_name(self.hand.get_links(), base_link)
        self.base_link_idx = self.hand.get_links().index(self.base_link)

        # Joints (USER order = config order)
        self.all_joints = get_active_joints(self.hand, joint_names)
        all_limits = [j.get_limits() for j in self.all_joints]

        self.joint_names = list(joint_names)
        self.user_idx_to_sim_idx = get_active_joint_indices(self.hand, self.joint_names)
        print("User-to-Sim Joint", self.user_idx_to_sim_idx)
        self.sim_idx_to_user_idx = [
            self.user_idx_to_sim_idx.index(i) for i in range(len(self.user_idx_to_sim_idx))
        ]
        print("Sim-to-User Joint", self.sim_idx_to_user_idx)

        self.joint_lower_limit = np.array([l[0][0] for l in all_limits], dtype=np.float32)
        self.joint_upper_limit = np.array([l[0][1] for l in all_limits], dtype=np.float32)
        print("Joint limits (user order):")
        print(self.joint_lower_limit, self.joint_upper_limit)

        # ---------- NEW: per-joint sign & offset (USER order) -----------------
        nd = len(self.joint_names) if self.joint_names else n_hand_dof
        if joint_sign is None:
            joint_sign = np.ones(nd, dtype=np.float32)
        if joint_offset is None:
            joint_offset = np.zeros(nd, dtype=np.float32)
        self.joint_sign = np.asarray(joint_sign, dtype=np.float32).reshape(-1)
        self.joint_offset = np.asarray(joint_offset, dtype=np.float32).reshape(-1)
        if self.joint_sign.size != nd or self.joint_offset.size != nd:
            raise ValueError(
                f"joint_sign ({self.joint_sign.size}) / joint_offset ({self.joint_offset.size}) "
                f"lengths must match number of user joints ({nd})"
            )
        print("[Retarget] joint_sign:", self.joint_sign)
        print("[Retarget] joint_offset (rad):", self.joint_offset)

        # Init mid pose
        mid_user = (self.joint_lower_limit + self.joint_upper_limit) / 2
        init_qpos = self.convert_user_order_to_sim_order(mid_user)
        self.hand.set_qpos(init_qpos)
        self.hand.set_qvel(0.0 * init_qpos)
        self.qpos_target = init_qpos

        for i, joint in enumerate(self.all_joints):
            print(i, self.joint_names[i], joint, self.joint_lower_limit[i], self.joint_upper_limit[i])
            joint.set_drive_property(kp, kd, force_limit=10)

    def __del__(self):
        del self.engine
        del self.scene

    def get_n_dof(self):
        return len(self.joint_lower_limit)

    def get_joint_limit(self):
        return self.joint_lower_limit, self.joint_upper_limit

    def initialize_keypoint(self, keypoint_link_names, keypoint_offsets):
        keypoint_links = [get_entity_by_name(self.hand.get_links(), link) for link in keypoint_link_names]
        print(keypoint_links)

        keypoint_links_id_dict = {
            link_name: (self.hand.get_links().index(keypoint_links[i]), i)
            for i, link_name in enumerate(keypoint_link_names)
        }
        self.keypoint_links = keypoint_links
        self.keypoint_links_id_dict = keypoint_links_id_dict
        self.keypoint_offsets = np.array(keypoint_offsets)

    def convert_user_order_to_sim_order(self, qpos_user):
        return np.asarray(qpos_user, dtype=np.float32)[self.sim_idx_to_user_idx]

    def keypoint_from_qpos(self, qpos_user, ret_vec=False):
        """
        qpos_user is in USER order. We compute FK and return points in base frame.
        """
        qpos_sim = self.convert_user_order_to_sim_order(qpos_user)
        self.pmodel.compute_forward_kinematics(qpos_sim)
        base_pose = self.pmodel.get_link_pose(self.base_link_idx)

        result = {}
        vec_result = []

        for m, (link_idx, i) in self.keypoint_links_id_dict.items():
            pose = self.pmodel.get_link_pose(link_idx)
            new_pose = sapien.Pose(
                p=pose.p + (pose.to_transformation_matrix()[:3, :3] @ self.keypoint_offsets[i].reshape(3, 1)).reshape(-1),
                q=pose.q,
            )

            x = (base_pose.inv() * new_pose).p  # convert to hand base frame.
            vec_result.append(x)
            result[m] = x

        if ret_vec:
            return np.array(vec_result)
        return result

    @staticmethod
    def build_from_config(config, **kwargs):
        """
        Build a kinematic model from user config.

        New optional keys in `config` (USER order):
          - "joint_sign":   list[float] of +1/-1 per joint
          - "joint_offset": list[float] (radians) per joint
          - "clamp_eps":    float (default 1e-3)
        """
        render = kwargs.get("render", False)
        urdf_path = config["urdf_path"]
        base_link = config["base_link"]
        joint_order = config["joint_order"]
        n_hand_dof = len(joint_order)

        joint_sign = config.get("joint_sign", None)
        joint_offset = config.get("joint_offset", None)
        clamp_eps = config.get("clamp_eps", 1e-3)

        model = HandKinematicModel(
            hand_urdf=urdf_path,
            render=render,
            n_hand_dof=n_hand_dof,
            base_link=base_link,
            joint_names=joint_order,
            joint_sign=joint_sign,
            joint_offset=joint_offset,
            clamp_eps=clamp_eps,
        )
        return model

    def get_viewer_env(self):
        return HandViewerEnv(self)

    def get_scene(self):
        return self.scene

    def get_renderer(self):
        return self.renderer

    def _apply_sign_offset(self, qpos_user):
        """
        Apply retargeting corrections in USER order.
        q_corr = q * sign + offset
        """
        q = np.asarray(qpos_user, dtype=np.float32).reshape(-1)
        if q.size != self.joint_sign.size:
            raise ValueError(f"Expected {self.joint_sign.size} joints, got {q.size}")
        return q * self.joint_sign + self.joint_offset

    def set_qpos_target(self, qpos_user):
        """
        Visualization control path:
          1) Apply sign/offset (USER order)  <-- LEFT/RIGHT fix lives here
          2) Clip to limits (USER order)
          3) Map to SIM order
          4) Send to joint drives
        """
        q_user_corr = self._apply_sign_offset(qpos_user)
        q_user_corr = np.clip(
            q_user_corr,
            self.joint_lower_limit + self.clamp_eps,
            self.joint_upper_limit - self.clamp_eps,
        )

        q_sim = self.convert_user_order_to_sim_order(q_user_corr)
        self.qpos_target = q_sim

        for i in range(len(q_sim)):
            self.all_joints[i].set_drive_target(self.qpos_target[i])


class HandViewerEnv:
    def __init__(self, model):
        scene = model.get_scene()
        scene.set_timestep(1 / 100.0)
        scene.set_ambient_light([0.5, 0.5, 0.5])
        scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
        scene.add_ground(altitude=0)

        viewer = Viewer(model.get_renderer())
        viewer.set_scene(scene)
        viewer.window.set_camera_position([0.1550926, -0.1623763, 0.7064089])
        viewer.window.set_camera_rotation([0.8716827, 0.3260138, 0.12817779, 0.3427167])
        viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)

        self.model = model
        self.scene = scene
        self.viewer = viewer

    def update(self):
        self.scene.step()
        self.scene.update_render()
        self.viewer.render()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--hand", type=str, default="allegro")
    args = parser.parse_args()

    # Load Hand Model
    config = get_config(args.hand)
    model = HandKinematicModel.build_from_config(config, render=True)
    viewer_env = model.get_viewer_env()

    # Control Loop
    n_dof = model.get_n_dof()
    dof_lower, dof_upper = model.get_joint_limit()

    steps = 0
    while True:
        viewer_env.update()
        steps += 1
        if steps % 30 == 0:
            targets = (
                np.random.uniform(0, 1, n_dof) * (dof_upper - dof_lower - 1e-7)
                + dof_lower
                + 1e-7
            )
            model.set_qpos_target(targets)