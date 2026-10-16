#!/usr/bin/env python3
"""
Dual-hand streamer (Conda / Python 3.11, no ROS):
- Right hand: ManusMocap on port 8765 → GeoRT(allegro_right) → PUB tcp://127.0.0.1:7777
- Left  hand: ManusMocap on port 8766 → GeoRT(allegro_left)  → PUB tcp://127.0.0.1:7778

Each message is JSON: {"stamp": float, "hand": "right"|"left", "qpos": [16 floats]}
"""

import argparse
import json
import time
import threading
from typing import Optional, Dict, Any, List

import zmq
import numpy as np

from geort.mocap.manus_mocap import ManusMocap
from geort import load_model, get_config

def _publisher_loop(name: str,
                    manus_port: int,
                    ckpt_tag: str,
                    hand_cfg_key: str,
                    pub_addr: str,
                    hz: float):
    """
    One loop per hand. Reads Manus mocap -> GeoRT forward -> PUB JSON.
    """
    print(f"[{name.upper()}] Init: mocap_port={manus_port}, ckpt={ckpt_tag}, cfg={hand_cfg_key}, pub={pub_addr}")

    # Load GeoRT model for this hand
    model = load_model(ckpt_tag)
    # (config is not strictly needed here since we don't visualize, but keep for parity)
    _ = get_config(hand_cfg_key)

    # Mocap
    mocap = ManusMocap(port=manus_port)

    # ZMQ PUB
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUB)
    sock.bind(pub_addr)
    sock.setsockopt(zmq.SNDHWM, 0)

    min_period = 1.0 / max(1e-6, hz)
    last_pub = 0.0

    try:
        while True:
            res: Dict[str, Any] = mocap.get()
            if res.get("status") == "quit":
                print(f"[{name.upper()}] Quit signal received.")
                break

            if res.get("status") == "recording" and res.get("result") is not None:
                now = time.time()
                if now - last_pub >= min_period:
                    # GeoRT forward pass: 21x3 canonical keypoints (res["result"]) -> 16-dof qpos
                    qpos: List[float] = np.asarray(model.forward(res["result"]), dtype=float).reshape(-1).tolist()

                    if len(qpos) != 16:
                        print(f"[{name.upper()}] WARNING: expected 16 joints, got {len(qpos)}. Skipping frame.")
                    else:
                        payload = {"stamp": now, "hand": name, "qpos": qpos}
                        sock.send_string(json.dumps(payload))
                        last_pub = now
                        # Debug (quiet by default)
                        # print(f"[{name.upper()}] PUB qpos: {qpos}")

            time.sleep(0.001)
    finally:
        try:
            sock.close(0)
        except Exception:
            pass
        # Do not term the shared instance here; other threads may still use it.

def main():
    parser = argparse.ArgumentParser()
    # Manus mocap input ports (right/left)
    parser.add_argument('--right_mocap_port', type=int, default=8765, help='Right-hand Manus ZMQ port')
    parser.add_argument('--left_mocap_port',  type=int, default=8766, help='Left-hand Manus ZMQ port')

    # GeoRT ckpts and config keys (override if needed)
    parser.add_argument('--right_ckpt', type=str, default='right_v1', help='GeoRT ckpt tag for RIGHT') #geort_3r
    parser.add_argument('--left_ckpt',  type=str, default='left_v2', help='GeoRT ckpt tag for LEFT')
    parser.add_argument('--right_cfg',  type=str, default='allegro_right', help='Config key for RIGHT')
    parser.add_argument('--left_cfg',   type=str, default='allegro_left',  help='Config key for LEFT')

    # Publisher addresses
    parser.add_argument('--right_pub', type=str, default='tcp://127.0.0.1:7777', help='ZMQ PUB addr for RIGHT')
    parser.add_argument('--left_pub',  type=str, default='tcp://127.0.0.1:7778', help='ZMQ PUB addr for LEFT')
    
    # Hand selection publish
    parser.add_argument('--hand_selc_pub', type=str, default='both', help='Which hand to publish: right, left, both')

    parser.add_argument('--hz', type=float, default=60.0, help='Max publish rate per hand')
    args = parser.parse_args()

    # Launch two publisher threads
    t_right = threading.Thread(
        target=_publisher_loop,
        args=("right", args.right_mocap_port, args.right_ckpt, args.right_cfg, args.right_pub, args.hz),
        daemon=True
    )
    
    t_left = threading.Thread(
        target=_publisher_loop,
        args=("left", args.left_mocap_port, args.left_ckpt, args.left_cfg, args.left_pub, args.hz),
        daemon=True
    )

    # Start threads (mapping-based, clearer and more extensible)
    _thread_map = {
        'right': (t_right,),
        'left': (t_left,),
        'both': (t_right, t_left),
    }
    threads_to_start = _thread_map.get(args.hand_selc_pub)
    if threads_to_start is None:
        print(f"[DUAL] Invalid hand_selc_pub option: {args.hand_selc_pub}. Choose from 'right', 'left', 'both'.")
        return
    for t in threads_to_start:
        t.start()

    print("[DUAL] Streamers running. Right→7777, Left→7778. Press Ctrl+C to stop.")
    try:
        while t_right.is_alive() or t_left.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[DUAL] KeyboardInterrupt. Exiting…")
    # Threads will exit when ManusMocap get() yields quit, or process is interrupted.

if __name__ == "__main__":
    main()
