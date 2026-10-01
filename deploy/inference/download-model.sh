#!/usr/bin/env bash
set -euo pipefail
model_dir="$HOME/robot/models/qwen3.5-9b"
mkdir -p "$model_dir"
cd "$model_dir"
base='https://huggingface.co/unsloth/Qwen3.5-9B-GGUF/resolve/3885219b6810b007914f3a7950a8d1b469d598a5'
for name in Qwen3.5-9B-Q5_K_M.gguf mmproj-F16.gguf; do
  aria2c --continue=true --max-connection-per-server=16 --split=16 \
    --file-allocation=none --summary-interval=60 --console-log-level=warn \
    --download-result=full --allow-overwrite=false --auto-file-renaming=false \
    --out="$name" "$base/$name"
done
