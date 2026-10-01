#!/usr/bin/env python3
"""Complete the one-off mini-PC model installation after downloads finish.

Run on the mini-PC as mrbobus. Starts only the local inference service;
never enables boot startup, ROS, navigation or motor processes.
"""
import base64
import hashlib
import json
import subprocess
import time
import urllib.request
from pathlib import Path

root = Path.home() / "robot"
runtime = root / "inference"
models = root / "models/qwen3.5-9b"
manifest = json.loads((models / "manifest.json").read_text())
deadline = time.monotonic() + 7200
while True:
    complete = all((models / f["rfilename"]).exists()
                   and (models / f["rfilename"]).stat().st_size == f["size"]
                   and not (models / (f["rfilename"] + ".aria2")).exists()
                   for f in manifest["files"])
    if complete:
        break
    if time.monotonic() > deadline:
        raise SystemExit("Download deadline expired; inference not started")
    time.sleep(15)
for item in manifest["files"]:
    with (models / item["rfilename"]).open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    if digest != item["lfs"]["sha256"]:
        raise SystemExit("SHA256 mismatch: " + item["rfilename"])
print("Both model SHA256 hashes verified", flush=True)
for gpu_layers in (0, 99):
    with (runtime / f"bench-ngl{gpu_layers}.json").open("w") as out:
        subprocess.run([str(runtime / "llama.cpp/build/bin/llama-bench"),
                        "-m", str(models / "Qwen3.5-9B-Q5_K_M.gguf"),
                        "-p", "128", "-n", "64", "-t", "8", "-r", "2",
                        "-ngl", str(gpu_layers), "-o", "json"],
                       stdout=out, check=True, timeout=600)
subprocess.run(["sudo", "-n", "systemctl", "start", "mrbobus-inference"], check=True)
deadline = time.monotonic() + 300
while True:
    try:
        with urllib.request.urlopen("http://127.0.0.1:8088/health", timeout=3) as response:
            if response.status == 200:
                break
    except Exception:
        if time.monotonic() > deadline:
            subprocess.run(["sudo", "-n", "systemctl", "stop", "mrbobus-inference"])
            raise SystemExit("Model server did not become healthy")
        time.sleep(2)
subprocess.run(["python3", str(runtime / "benchmark_mission_model.py"),
                "--output", str(runtime / "mission-results.json")], check=True, timeout=1800)
results = json.loads((runtime / "mission-results.json").read_text())
print(f"Synthetic mission smoke: {results['passed']}/{len(results['cases'])}", flush=True)
if (runtime / "field-catalog.json").exists() and (runtime / "field-cases.json").exists():
    subprocess.run(["python3", str(runtime / "benchmark_mission_model.py"),
        "--catalog", str(runtime / "field-catalog.json"),
        "--cases", str(runtime / "field-cases.json"),
        "--output", str(runtime / "field-mission-results.json")], check=True, timeout=1800)
image = runtime / "vision-smoke.jpg"
if image.exists():
    payload = {
        "model": "robot-model", "temperature": 0, "max_tokens": 128,
        "chat_template_kwargs": {"enable_thinking": False},
        "messages": [{"role": "user", "content": [
            {"type": "image_url", "image_url": {"url":
             "data:image/jpeg;base64," + base64.b64encode(image.read_bytes()).decode()}},
            {"type": "text", "text": "Кратко опиши, что перед роботом. "
             "Виден ли на поверхности полигона игрушечный пострадавший или QR-код? "
             "Не считай людей на настенных плакатах пострадавшими. "
             "Если не можешь уверенно определить, так и скажи."},
        ]}],
    }
    start = time.monotonic()
    request = urllib.request.Request("http://127.0.0.1:8088/v1/chat/completions",
        data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=300) as response:
        result = json.load(response)
    result["wall_seconds"] = time.monotonic() - start
    result["note"] = "One negative real camera frame; not a detection accuracy evaluation"
    (runtime / "vision-smoke-result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print("Vision smoke completed", flush=True)
print("Inference remains running on localhost:8088; boot startup is NOT enabled", flush=True)
