#!/usr/bin/env bash
# Live dual-hand teleoperation: two Manus gloves -> GeoRT (left + right) -> both Allegro hands.
# Opens one terminal tab per stage. Run from anywhere; paths are resolved from this repo.
#
#   GEORT_ENV=geort ROS_SETUP=/opt/ros/humble/setup.bash bash scripts/run_dual_teleop.sh
#
# Stages: (1) Manus SDK ROS 2 clients  (2) keypoint FK -> ZMQ :8765/:8766
#         (3) GeoRT streamer -> ZMQ :7777/:7778  (4) ROS 2 bridge -> /{ra,la}_/allegroHand_*/joint_cmd
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
GEORT_ENV="${GEORT_ENV:-geort}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
CONDA_SH="${CONDA_SH:-$(conda info --base 2>/dev/null)/etc/profile.d/conda.sh}"
ENV_SETUP="source $ROS_SETUP; export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-10}; \
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}; export RMW_CYCLONEDDS_USE_ICEORYX=0; \
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; export RMW_CYCLONEDDS_LOG_LEVEL=error"

gnome-terminal --tab --title="1 Manus clients" -- bash -c "$ENV_SETUP; cd $REPO/geort/mocap; \
  source install/setup.bash; ros2 run manus_client manus_right & ros2 run manus_client manus_left; exec bash"
gnome-terminal --tab --title="2 Mocap FK" -- bash -c "$ENV_SETUP; cd $REPO/geort/mocap; \
  source install/setup.bash; python3 manus_mocap_core_left.py & python3 manus_mocap_core.py; exec bash"
gnome-terminal --tab --title="3 GeoRT streamer" -- bash -c "source $CONDA_SH; conda activate $GEORT_ENV; \
  cd $REPO; python geort/mocap/geort_streamer_dual.py; exec bash"
gnome-terminal --tab --title="4 Allegro bridge" -- bash -c "$ENV_SETUP; cd $REPO; \
  python3 geort/mocap/ros_joint_bridge_dual.py --right_addr tcp://127.0.0.1:7777 --left_addr tcp://127.0.0.1:7778 \
  --right_topic /ra_/allegroHand_0/joint_cmd --left_topic /la_/allegroHand_1/joint_cmd --hz 60 --clamp; exec bash"
