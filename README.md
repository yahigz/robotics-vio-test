## Evaluation & tools

Environment: `pixi install` (python 3.11, numpy/scipy/matplotlib/opencv, `evo`, `rosbags`; no ROS needed).
All tasks run with `MPLBACKEND=Agg`. Data layout and result paths: see `CONTRACT.md`.

| task | what it does |
|---|---|
| `pixi run eval --gt GT.txt --est LABEL=traj.txt [LABEL2=...] --dataset euroc --seq MH_01_easy` | evo APE (SE(3) Umeyama, no scale; `--sim3` for scale) + RPE (translation and rotation, `--delta` meters, default 10) + KITTI-style per-segment RPE (`--segments 10 20 50 100`, only lengths shorter than the GT path). Association by timestamp (`--t-max-diff` 0.02 s, `--t-offset`). Writes `results/summary/<dataset>/<seq>/{metrics.json,metrics.csv,*.png,*.pdf}` and prints a markdown table. |
| `pixi run collect [--root results/summary]` | aggregates all `metrics.json` into `results/summary/all.{md,csv}` |
| `pixi run bag-filter in.bag out.bag --topics /cam0/image_raw /imu0 [--decompress]` | streaming topic filter for ROS1 bags; `--decompress` turns CompressedImage into Image |
| `pixi run bag-to-asl in.bag out_dir --cam0 /cam0/image_raw [--cam1 ...] --imu /imu0` | ROS1 bag to EuRoC/ASL folder (`out_dir/mav0/...`) |
| `pixi run gt-to-tum euroc data.csv out.txt` / `csv ...` / `bag in.bag out.txt --topic /pose` | ground truth to TUM format (seconds) |
| `pixi run test-eval` | self-test: perturbs GT (drift + noise + offset frame) and runs `eval` into `results/summary/selftest/` |

Trajectory format: TUM `t tx ty tz qx qy qz qw`, `t` in seconds, same clock as GT.
