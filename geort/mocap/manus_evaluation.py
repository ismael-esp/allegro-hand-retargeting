# manus_evaluation.py
# Evaluate either hand by choosing config and ZMQ port.

from geort.mocap.manus_mocap import ManusMocap
from geort.env.hand import HandKinematicModel
from geort import load_model, get_config
import argparse


def parse_args():
    parser = argparse.ArgumentParser(
        description="Evaluate a GeoRT model with the SAPIEN hand simulator using Manus ZMQ mocap"
    )
    parser.add_argument('--hand', type=str, default='allegro_left',
                        help="Hand config key, e.g., allegro_right or allegro_left")
    parser.add_argument('--ckpt_tag', type=str, default='left_v3',
                        help="GeoRT checkpoint tag (model weights)")
    parser.add_argument('--port', type=int, default=8766,
                        help="ZMQ port for Manus mocap (example: right=8765, left=8766)")
    parser.add_argument('--no-render', action='store_true',
                        help="Disable rendering of the hand simulator")
    return parser.parse_args()


def main():
    args = parse_args()

    # Load GeoRT model
    model = load_model(args.ckpt_tag)

    # Motion Capture (choose port)
    mocap = ManusMocap(port=args.port)

    # Robot Simulation.
    config = get_config(args.hand)
    hand = HandKinematicModel.build_from_config(config, render=not args.no_render)
    viewer_env = hand.get_viewer_env()

    # Run main loop
    while True:
        # update viewer (no-op if rendering disabled)
        viewer_env.update()
        result = mocap.get()
        if result.get('status') == 'recording' and result.get('result') is not None:
            qpos = model.forward(result['result'])
            hand.set_qpos_target(qpos)
        if result.get('status') == 'quit':
            break


if __name__ == '__main__':
    main()