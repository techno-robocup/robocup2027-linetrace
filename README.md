# RoboCup 2027 — C++ ML Line-Trace System

End-to-end machine-learning line tracing in C++: drive the robot RC-style with a
gamepad, record `(image, motor-command)` pairs, train a small CNN (LibTorch),
and run it on the robot to follow the line autonomously.

```
  LAPTOP (control_station)            RASPBERRY PI (collector / executor)        TRAINING PC
  gamepad ──UDP control 50Hz──▶ collector ──▶ ESP32 (UART 115200)
          ◀──UDP JPEG preview── │ libcamera ──▶ frames ──▶ Pi disk (jpg+labels.csv)
                                                              │ push_session (TCP, bulk)
                                                              └──────────────▶ transfer_node server
                                                                               trainer ─▶ model.pt
                                  executor ◀── model.pt ◀──────────────────────────────┘
                                  libcamera ─▶ infer ─▶ MOTOR via ESP32
```

## Components

| Path | What | Deps | Verified |
|---|---|---|---|
| `libs/common` | config, label codec, arcade mixing, UDP wire protocol (header-only) | — | ✅ unit test |
| `libs/common_torch` | `preprocess()` + `LineNet` model (shared by trainer & executor) | Torch, OpenCV | ✅ unit test |
| `libs/esp32_driver` | threaded UART driver + `uart_probe` tool | POSIX | ✅ pty-loopback test |
| `libs/transfer_client` | client for the transfer protocol | POSIX | ✅ test vs `transfer_node` |
| `libs/camera` | `FrameSource`: OpenCV/synthetic + native libcamera | OpenCV, libcamera? | ✅ synthetic; 🔧 libcamera on Pi |
| `libs/netutil` | UDP helpers | POSIX | ✅ via integration test |
| `apps/trainer` | LibTorch training → `linetrace_model.pt` + `model_info.json`; `model_probe` | Torch, OpenCV | ✅ trained on 21k frames |
| `apps/collector` | record dataset on the Pi; UDP control + preview; ESP32 drive | OpenCV, (libcamera) | ✅ synthetic-cam end-to-end |
| `apps/control_station` | gamepad → UDP control; preview window | OpenCV, (SDL2) | ✅ scripted integration |
| `apps/executor` | load model → infer on frames → drive ESP32 | Torch, OpenCV, (libcamera) | ✅ ~56 fps on synthetic |
| `apps/push_session` | upload a session to the training PC | POSIX | ✅ test vs `transfer_node` |

🔧 = needs on-device build/verification (real libcamera, ESP32, SDL2 gamepad, GUI preview).

## Build

LibTorch is taken from the vendored `../yolo_cpp/libtorch` (2.11.0+cpu) by
default; override with `-DLIBTORCH_DIR=...`. Build only what each machine needs:

```bash
# Training PC (LibTorch + OpenCV)
cmake -S . -B build -G Ninja -DBUILD_TRAINER=ON
cmake --build build                       # -> trainer, model_probe, transfer_client, tests

# Raspberry Pi (OpenCV [+ libcamera])
cmake -S . -B build -G Ninja -DBUILD_TRAINER=OFF -DBUILD_TESTS=OFF \
      -DBUILD_COLLECTOR=ON -DBUILD_EXECUTOR=ON
cmake --build build                       # -> collector, executor, uart_probe, push_session

# Laptop control station (OpenCV + SDL2)
sudo apt install libsdl2-dev              # for a real gamepad
cmake -S . -B build -G Ninja -DBUILD_TRAINER=OFF -DBUILD_TESTS=OFF -DBUILD_CONTROL_STATION=ON
cmake --build build                       # -> control_station
```

Run anything Torch-linked with `LD_LIBRARY_PATH=../yolo_cpp/libtorch/lib`.
`ctest` (in `build/`) runs the PC unit tests.

## Workflow

1. **Train** (PC): `trainer --data ../linetrace_data --out models --epochs 40`
   - Trains `LineNet` on the labels.csv sessions; reports val MAE in motor-command
     units (range -8000..8000, neutral 0); writes `models/linetrace_model.pt` +
     `model_info.json`.
   - `--sensors` adds IMU/ultrasonic fusion; `--grayscale`, `--target-space ts`,
     `--limit N` (subset) also available.
   - Verify load+infer: `model_probe models/linetrace_model.pt <frame.jpg>`.
2. **Collect more data** (robot + laptop):
   - Robot: `collector --camera libcamera --dataset-root collected`
     (desktop test: `--camera synthetic --no-esp`).
   - Laptop: `control_station --robot <pi-ip>` (drive with the gamepad; **A** =
     record toggle, **B**/bumpers = e-stop). No gamepad? `--input mouse`: hold
     the left button on the preview window and drag (a continuous virtual
     stick — up/down = throttle, left/right = steer, release = stop; keys:
     `r` = record toggle, space = e-stop toggle, `q` = quit).
3. **Ship data** (robot → PC):
   - PC: `transfer_node server 9000 datasets`
   - Robot: `push_session <pc-ip> 9000 collected/session_XXXX`
   - Retrain on the combined data.
4. **Run autonomously** (robot): `executor --model linetrace_model.pt --camera libcamera`.
   - Live view of the model's predictions: add `--preview-to <pc-ip>` and run
     `control_station --view-only` on the PC — streams the camera frames with
     the predicted L/R and a steering bar burned in (15 fps preview; safe to
     combine with `--no-esp` for a motors-off dry run).

## ESP32 firmware

The firmware baud was raised **4800 → 115200** (`robocup2026-esp32-program`:
`lib/serialio/serialio.cpp` + `platformio.ini`) for ~4 ms motor round-trips and
smooth ~50 Hz control. Reflash the ESP32, then verify with `uart_probe`.

## Status & results

- Single-frame `LineNet` trained on the existing ~21k frames reaches **val MAE
  ≈ 70 PWM** (40 epochs, CPU), vs a **constant-mean baseline of 99 PWM** — so it
  is genuinely using the image. **But** per-frame probes on held-out val frames
  show it discriminates only **weakly**: forward frames predict ~1668/1668
  correctly, but left/right turns produce muted, noisy outputs that don't cleanly
  reproduce the ±200 turn differential. **It is not yet good enough for reliable
  autonomous following.** Next steps to beat the prior prototype's ≈63-PWM
  sensors+memory result: enable `--sensors`, add the temporal-memory (GRU) head,
  and collect more *continuous* gamepad data (the 2026 data is keyboard-quantized
  into ~9 discrete maneuvers).
- Label = the continuous `(left, right)` motor command per frame; `labels.csv`
  is schema-compatible with the 2026 dataset so existing data is reusable.

### ⚠️ Orientation consistency (verify before trusting on-robot inference)

The model must see frames in the **same orientation** at training and inference.
The pipeline is self-consistent for **newly collected** data: the collector's
`FrameSource` applies `rotate180`, stores oriented frames; the trainer reads them
without rotating; the executor's `FrameSource` also applies `rotate180` and its
preprocessing does not. **The existing `linetrace_data` orientation is unverified**
— if those 21k frames were stored *un-rotated*, a model trained on them will be
fed upside-down input by the executor and fail on the robot. Before deploying a
model trained on the old data, confirm its orientation matches the collector's
(or retrain on freshly collected data).

## On-device verification checklist (next)

- [ ] `uart_probe` round-trips `MOTOR`/`GET *` at 115200 on the real ESP32.
- [ ] `collector --camera libcamera`: confirm the lores pixel format
      (RGB888 vs YUV420 — see `libs/camera/src/libcamera_source.cpp`).
- [ ] Drive with a real SDL2 gamepad; confirm preview window + recording.
- [ ] Benchmark `executor` inference FPS on the Pi (target ≥30; else grayscale /
      smaller input / quantize). Confirm autonomous line following.
```
