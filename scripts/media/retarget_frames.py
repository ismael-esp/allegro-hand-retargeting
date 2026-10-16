"""Step A (SAPIEN 2.2.2): Manus keypoints -> GeoRT model -> joint targets exactly as HandKinematicModel applies them."""
import argparse, numpy as np
from geort import load_model, get_config
from geort.env.hand import HandKinematicModel
p = argparse.ArgumentParser()
p.add_argument("--data"); p.add_argument("--ckpt"); p.add_argument("--hand"); p.add_argument("--out")
p.add_argument("--start", type=int); p.add_argument("--n", type=int); p.add_argument("--stride", type=int, default=2)
a = p.parse_args()
kp = np.load(a.data)[a.start:a.start + a.n * a.stride:a.stride].astype(np.float32)
model = load_model(a.ckpt); cfg = get_config(a.hand)
h = HandKinematicModel.build_from_config(cfg)
names = [j.get_name() for j in h.hand.get_active_joints()]
qs = []
for k in kp:
    q = np.clip(h._apply_sign_offset(model.forward(k)), h.joint_lower_limit + 1e-3, h.joint_upper_limit - 1e-3)
    qs.append(h.convert_user_order_to_sim_order(q))
np.savez(a.out, kp=kp, q_sim=np.array(qs), names=np.array(names), urdf=cfg["urdf_path"])
print("saved", a.out, len(qs))
