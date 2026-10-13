# vio-bench: VIO methods across the filter / optimization / learned lineage

## Quick start
```bash
# 0. one-time setup (user): ClearML credentials, GitHub token for Colab, Colab GPU runtime + secrets -> docs/ACCESS_SETUP.md
# 0b. DBA-Fusion launch checklist (Colab) -> docs/RUN_COLAB.md
pixi install                           # eval/tools environment (python 3.11, evo, rosbags, clearml)
docker compose build                   # openvins + orbslam3 images (once, ~10 min each)

# data (not in git; ~3-4 GB per ROVER sequence: T265 left camera + IMU + GT, fetched by HTTP range requests, needs curl)
mkdir -p data/rover && cp tools/rover/calib/calib_t265.yaml data/rover/
for s in garden_small_2024-05-29_1 campus_small_2024-02-19 park_2023-07-31; do
  python3 tools/rover/fetch_subset.py fetch $s && pixi run python tools/rover/convert_asl.py $s; done
pixi run python tools/fetch_euroc_hf.py --seq MH_01_easy --data-root data   # EuRoC sanity sequence

pixi run check-access                  # 1. checklist: docker, images, pixi env, data, GitHub remote, ~/clearml.conf, MCP servers
pixi run run-local                     # 2. OpenVINS + ORB-SLAM3 (x3 runs) on ROVER (3 seqs) + EuRoC MH_01, evaluation, tables
                                       #    options: --seqs garden,park --methods openvins --force --runs 3 --clearml
git push                               # 3. the private repo is the code source for Colab (token in Colab Secrets)
#  4. Colab: open methods/dbafusion/dbafusion_colab.ipynb, GPU runtime, edit SEQUENCES in cell 0, Run all
#     (code: git clone with GITHUB_TOKEN; data: downloaded to Colab's local disk; results: ClearML artifacts; no Google Drive needed)
pixi run fetch-colab                   # 5. pull the ClearML artifacts -> results/dbafusion/..., re-evaluate: DBA-Fusion joins the same tables and plots
```
All scripts are idempotent (finished runs are skipped unless `--force`), log to `results/logs/`, and can be restarted after an interruption.

### Methods, branches, datasets
| method | branch of VIO history | where it runs | config / entry point | datasets |
|---|---|---|---|---|
| OpenVINS v2.7 (MSCKF) | filter-based | local Docker (arm64, ROS1) | `methods/openvins/run_from_asl.sh <ds> <seq> <config>` | ROVER T265 mono-inertial (`rover_t265_mono`), EuRoC (`euroc_mav_mono`) |
| ORB-SLAM3 v1.0 | optimization-based (keyframe BA) | local Docker (arm64) | `docker compose run --rm orbslam3 <ds> <seq> mono_inertial <settings> [--runs N] [--max-frames N]` | ROVER (`rover_t265_mono_inertial.yaml`), EuRoC |
| DBA-Fusion (RA-L 2024) | learned / hybrid (dense BA + IMU) | Google Colab GPU + ClearML | `methods/dbafusion/dbafusion_colab.ipynb` | ROVER (rectified to pinhole in Colab), EuRoC |

Datasets: ROVER (T265 fisheye + BMI055 IMU; garden_small_2024-05-29_1, campus_small_2024-02-19, park_2023-07-31; GT = total-station prism **position only**, ~2 Hz,
prism ~0.45 m from the IMU) and EuRoC MH_01_easy (sanity). ROVER is evaluated with `--trans-only --interp-est` and the prism lever arm (see below).

### Results (full table: `results/summary/all.md`)
ROVER, mono-inertial, APE RMSE [m] with SE(3) / Sim(3) alignment, s* = Sim(3) scale (>1: path underestimated), RPE translation RMSE over 10 m.
ORB-SLAM3: median-APE run of 3. DBA-Fusion: one run on Colab T4.

| seq (GT path) | method | APE SE(3) | APE Sim(3) | s* | RPE@10 m |
|---|---|---|---|---|---|
| garden_small (121 m) | OpenVINS | 0.84 | 0.60 | 0.90 | 0.98 |
| | ORB-SLAM3 (runs 1.38 / 1.53 / 2.55) | 1.53 | 1.16 | 1.24 | 2.51 |
| | DBA-Fusion | 0.78 | 0.72 | 1.06 | 1.02 |
| campus_small (150 m) | OpenVINS | 1.83 | 1.64 | 0.89 | 1.11 |
| | ORB-SLAM3 (runs 2.41 / 2.84 / 3.13) | 2.84 | 2.25 | 1.37 | 4.51 |
| | DBA-Fusion | 0.64 | 0.56 | 1.05 | 0.77 |
| park (164 m) | OpenVINS | 1.68 | 1.64 | 0.96 | 1.42 |
| | ORB-SLAM3 (runs 1.78 / 1.99 / 2.02) | 1.99 | 0.76 | 1.30 | 2.67 |
| | DBA-Fusion | 5.71 | 5.16 | 0.71 | 3.37 |

DBA-Fusion on park diverges for the first ~60 s after VI initialization (slow start, weak IMU excitation; peak APE 39 m).
EuRoC MH_01_easy sanity (APE SE(3)): OpenVINS stereo 0.098, OpenVINS mono 0.129, ORB-SLAM3 mono-inertial 0.087.

### Report figures
`pixi run figures` -> `report/overleaf/` (`figures/*.pdf`, `tables/results.tex`, `main.tex` skeleton for Overleaf, pdfLaTeX).
Same evaluation as `eval/evaluate.py`; zip for upload: `cd report/overleaf && zip -r ../vio-report-overleaf.zip main.tex figures tables`.

### Results layout
```
results/<method>/<dataset>/<seq>/traj_tum.txt    TUM "t tx ty tz qx qy qz qw" (IMU pose), log.txt     (openvins: .../<seq>/<config>/traj_tum.txt;
                                                  orbslam3: results/orbslam3_<mode>/..., traj_tum_run<K>.txt for every run)
results/summary/<dataset>/<seq>/                 metrics.json/csv, trajectory_xy.png, ape_*.png, rpe_boxplot.png, ... (all methods of that sequence)
results/summary/all.{md,csv}                     one table over all datasets/sequences/methods (pixi run collect)
results/logs/                                    run logs
```

## Evaluation & tools

Environment: `pixi install` (python 3.11, numpy/scipy/matplotlib/opencv, `evo`, `rosbags`, `clearml`; no ROS needed).
All tasks run with `MPLBACKEND=Agg`. Data layout and result paths: see `CONTRACT.md`.

| task | what it does |
|---|---|
| `pixi run eval --gt GT.txt --est LABEL=traj.txt [LABEL2=...] --dataset euroc --seq MH_01_easy` | evo APE (SE(3) Umeyama, no scale; `--sim3` for scale) + RPE (translation and rotation, `--delta` meters, default 10) + KITTI-style per-segment RPE (`--segments 10 20 50 100`, only lengths shorter than the GT path). Association by timestamp (`--t-max-diff` 0.02 s, `--t-offset`). Writes `results/summary/<dataset>/<seq>/{metrics.json,metrics.csv,*.png,*.pdf}` and prints a markdown table. |
| `pixi run eval ... --trans-only --interp-est --est-lever-arm X Y Z` (or `--lever-arm-calib data/rover/calib_t265.yaml`) | For position-only GT (ROVER prism): no rotation metrics (N/A), RPE on world-frame displacements; the estimated IMU pose is moved to the prism, `p_prism = p + R*t` (t = prism in IMU frame, `prism.position_in_imu` of the calib yaml), before alignment; `--interp-est` lerps/slerps the estimate at the sparse GT timestamps (default association is nearest within `--t-max-diff`); `--clearml` reports metrics + plots to ClearML (project `vio-bench`, task `<method>/<dataset>/<seq>`; warns and continues if clearml is missing/unconfigured). |
| `pixi run collect [--root results/summary]` | aggregates all `metrics.json` into `results/summary/all.{md,csv}` |
| `pixi run bag-filter in.bag out.bag --topics /cam0/image_raw /imu0 [--decompress]` | streaming topic filter for ROS1 bags; `--decompress` turns CompressedImage into Image |
| `pixi run bag-to-asl in.bag out_dir --cam0 /cam0/image_raw [--cam1 ...] --imu /imu0` | ROS1 bag to EuRoC/ASL folder (`out_dir/mav0/...`) |
| `pixi run asl-to-bag data/euroc/MH_01_easy/asl out.bag [--cam1]` | ASL folder to ROS1 bag (`/cam0/image_raw`, `/cam1/image_raw`, `/imu0`) |
| `pixi run gt-to-tum euroc data.csv out.txt` / `csv ...` / `bag in.bag out.txt --topic /pose` | ground truth to TUM format (seconds) |
| `pixi run run-local` / `fetch-colab` / `check-access` | Orchestration scripts in `scripts/` (see Quick start). |
| `pixi run figures` | paper-sized figures + LaTeX table for the report (`report/make_figures.py` -> `report/overleaf/`) |
| `pixi run test-eval` | self-test: perturbs GT (drift + noise + offset frame) and runs `eval` into `results/summary/selftest/` |

Trajectory format: TUM `t tx ty tz qx qy qz qw`, `t` in seconds, same clock as GT.
