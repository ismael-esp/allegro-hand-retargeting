#!/usr/bin/env python3
"""Per-finger flexion agreement between recorded human hands and GeoRT's Allegro output.

For every sampled frame, human flexion is the sum of the two bend angles along a finger's keypoint
chain, and robot flexion is the sum of that finger's three flexion joints. Reports Pearson r per finger
and writes a scatter plot + CSV. Only needs the trained models (no simulator).

    python scripts/eval_flexion_correlation.py            # after scripts/download_data.sh
"""
import argparse
import os

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from geort import load_model  # noqa: E402

CHAINS = {"thumb": [1, 2, 3, 4], "index": [5, 6, 7, 8], "middle": [9, 10, 11, 12], "ring": [13, 14, 15, 16]}
ROBOT = {"thumb": [13, 14, 15], "index": [1, 2, 3], "middle": [5, 6, 7], "ring": [9, 10, 11]}
COLOR = {"thumb": "#e15759", "index": "#4e79a7", "middle": "#59a14f", "ring": "#f28e2b"}


def flexion(k, chain):
    v = [k[chain[i + 1]] - k[chain[i]] for i in range(3)]
    return sum(np.arccos(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9), -1, 1))
               for a, b in zip(v[:-1], v[1:]))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--right", default="data/manus_data_right.npy:right_v1", help="recording:checkpoint_tag")
    p.add_argument("--left", default="data/manus_left_03.npy:left_v2", help="recording:checkpoint_tag")
    p.add_argument("--every", type=int, default=10, help="use every N-th frame")
    p.add_argument("--out", default="docs/results")
    a = p.parse_args()
    os.makedirs(a.out, exist_ok=True)

    rows = []
    fig, axs = plt.subplots(2, 4, figsize=(15, 6.2), dpi=130)
    for r_, (hand, spec) in enumerate((("right", a.right), ("left", a.left))):
        path, tag = spec.split(":")
        kp = np.load(path)
        idx = np.arange(0, len(kp), a.every)
        model = load_model(tag)
        q = np.array([model.forward(kp[i].astype(np.float32)) for i in idx])
        for c_, finger in enumerate(CHAINS):
            h = np.array([flexion(kp[i], CHAINS[finger]) for i in idx])
            r = q[:, ROBOT[finger]].sum(1)
            cc = float(np.corrcoef(h, r)[0, 1])
            rows.append((hand, tag, finger, len(idx), cc))
            ax = axs[r_, c_]
            ax.scatter(h, r, s=3, alpha=0.25, color=COLOR[finger])
            ax.set_title(f"{hand} {finger}   r = {cc:.2f}", fontsize=11)
            ax.set_xlabel("human finger flexion [rad]")
            if c_ == 0:
                ax.set_ylabel("Allegro finger flexion [rad]")
            for s in ("top", "right"):
                ax.spines[s].set_visible(False)
            print(f"{hand:5s} {finger:6s} r = {cc:.3f}  ({len(idx)} frames)")
    fig.suptitle("Learned retargeting keeps finger flexion monotonic (recorded Manus sessions → GeoRT models)",
                 fontsize=13)
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, "flexion_correlation.png"))
    with open(os.path.join(a.out, "flexion_correlation.csv"), "w") as f:
        f.write("hand,checkpoint,finger,frames,pearson_r\n")
        for row in rows:
            f.write("{},{},{},{},{:.3f}\n".format(*row))


if __name__ == "__main__":
    main()
