#!/usr/bin/env bash
# Download the recorded Manus glove sessions used to train / evaluate the models into data/.
#   bash scripts/download_data.sh
set -euo pipefail
cd "$(dirname "$0")/.."
URL="https://github.com/ismael-esp/allegro-hand-retargeting/releases/download/v1.0/manus_recordings.tar.gz"
curl -fL "$URL" -o /tmp/manus_recordings.tar.gz
tar -xzf /tmp/manus_recordings.tar.gz -C data
rm -f /tmp/manus_recordings.tar.gz
ls -lh data
