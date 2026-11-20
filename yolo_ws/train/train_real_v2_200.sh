#!/usr/bin/env bash
set -euo pipefail

training_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$training_dir"

exec /home/wego/miniconda3/envs/dl_env/bin/python train_baseline_model.py \
  --data "$training_dir/dataset.yaml" \
  --model_pt "$training_dir/../yolo26n.pt" \
  --name autorace_real_v2_20261119_200e_noflip \
  --epoch 200 \
  --patience 100 \
  --bs 8 \
  --device 0
