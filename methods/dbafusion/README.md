# DBA-Fusion branch (learned / hybrid VIO) -- Colab + ClearML

DBA-Fusion (Zhou et al., RA-L 2024; https://github.com/GREAT-WHU/DBA-Fusion): DROID-SLAM-style deep dense bundle
adjustment fused with IMU preintegration in a GTSAM factor graph. Here: **monocular VIO, no GNSS/odometry**.

## 1. Feasibility: can it run on an Apple M4 (PyTorch MPS)?

**Verdict: no, not without a rewrite of the core. It needs an NVIDIA GPU, so we use Colab.**
(Checked in the code at the pinned commit; DROID-SLAM and DPVO share the same structure.)

* `droid_backends` (DBA-Fusion `src/`: `droid_kernels.cu` 1955 lines, `correlation_kernels.cu`, `altcorr_kernel.cu`) is a
  CUDA-only torch extension, built by `CUDAExtension` + nvcc. It holds the Gauss-Newton dense BA (Schur complement on GPU),
  the correlation volume lookup, `iproj`, `depth_filter`, frame-distance... There is no CPU or Metal implementation.
  nvcc does not exist on macOS.
* `lietorch` (SE3/Sim3 ops): has a CPU implementation (`lietorch_cpu.cpp`) and a CUDA one, but **no MPS dispatch**; the C++
  dispatcher in `lietorch.cpp` falls through to `return a;` for any other device, i.e. an MPS tensor would silently get
  wrong (unchanged) results rather than an error.
* `torch_scatter`: CPU/CUDA only (trivially replaceable by `scatter_add_`; we do ship such a shim, `scripts/shims`).
* The Python code hard-codes `cuda`: `DBAFusion.load_weights` -> `.to("cuda:0")`, `DepthVideo` allocates its buffers with
  `device="cuda"` (+ `.share_memory_()`), `CovisibleGraph(device="cuda:0")`, `torch.cuda.*` calls. It also uses float64
  tensors (`tstamp`), which MPS does not support.
* Pure-CPU fallback is conceivable (lietorch CPU + rewriting the BA/correlation kernels in torch), but that is a research port
  (thousands of lines of kernels), would be very slow, and would change numerics; not worth it for a benchmark.
* GTSAM (custom fork) itself is plain C++ and would build on arm64, so only the CUDA part is the blocker.
* **Community ports:** I searched GitHub (repository and issue search for DROID-SLAM / DPVO / lietorch with "mps", "macOS",
  "Apple silicon") and the web and **found none**. I cannot prove none exists; I am not recommending any. Do not count on one.

## 2. What is pinned

| Item | Pin |
|---|---|
| DBA-Fusion | `2c658a3be747247ed0b17af4aef42831ed5e7310` (main, 2025-03-11) incl. submodules `thirdparty/lietorch` @ `0fa9ce8ffca86d985eca9e189a99690d6f3d4df6`, `thirdparty/eigen` @ `a2f8eba0` |
| Custom GTSAM fork | https://github.com/yuxuanzhou97/gtsam @ `c93ea5ba01dec1bf909ba9392fbe84975cb75a82` (2024-02-15, the version DBA-Fusion was released with; HEAD of the fork moved on to MASt3R-Fusion in 2025 -- the APIs DBA-Fusion uses were checked to exist in this commit) |
| Weights | `droid.pth` = DROID-SLAM TartanAir checkpoint (unchanged model), Google Drive id `1PpqVt1H4maBa_GbPJp4NwxRsd9jk-elh` (from upstream README) |
| torch | **Colab's preinstalled torch** (not touched); extensions are built against it |

Upstream targets python 3.10 / torch 1.11+cu113 / numpy 1.25 / torch_scatter 2.0.9. Our plan to run on current Colab:

1. **lietorch + droid_backends**: built from source against Colab's torch with `scripts/patch_build.py`, which (a) writes two clean
   setup files whose `-gencode` is the *detected* GPU capability + PTX (upstream stops at sm_86 without PTX, so L4/sm_89 would fail),
   (b) replaces deprecated `Tensor.type()` in `AT_DISPATCH_*` by `scalar_type()`, (c) adds `-std=c++17`. If the build fails with the
   bundled (old, pre-3.4) Eigen, the notebook retries with system Eigen 3.4 (`libeigen3-dev`). Wheels cached on Drive, key = python/torch/arch.
2. **Custom GTSAM**: built from source (cmake, Python wrapper, no TBB, no march=native so the cache is portable, no unstable/tests/examples),
   first with system pybind11>=2.13 (the fork's bundled pybind11 is from 2023 and may not support Colab's python), falling back to the bundled one.
   `make install` + `make python-install`; the result (site-packages `gtsam` + `/usr/local/lib/libgtsam*`) is tarred to Drive by `scripts/gtsam_cache.py`.
3. **torch_scatter**: not installed; pure-torch shim (`scripts/shims/torch_scatter.py`, unit-tested against a loop reference).
4. **numpy-quaternion, open3d, h5py**: not needed (our driver replaces the demo script).
5. `evo`, `clearml`, `gdown`, `pyyaml` via pip.

## 3. Input/output format (from reading the upstream code)

DBA-Fusion has no dataset reader of its own; each `demo_vio_*.py` builds, in memory:
* images: stream of `(t_seconds, uint8 [1,3,H,W], [fx,fy,cx,cy])`, undistorted, resized to ~384x512 px area with H,W multiples of 8
  (intrinsics rescaled), CLAHE applied on channel 0 and replicated. Camera frame stride (`--stride`) sets the keyframe-candidate rate.
* IMU: array `[t_s, gx, gy, gz (**deg/s**), ax, ay, az (m/s^2)]` (the demo multiplies rad/s by 180/pi; the frontend converts back).
  EuRoC `imu0/data.csv` has exactly this column order.
* all camera timestamps (seconds), used for the full-rate IMU-propagated output between keyframes.
* extrinsics `video.Ti1c`/`Tbc` = **T_imu_cam** (camera -> IMU/body; EuRoC `T_BS`), IMU noise `[acc, gyro, acc_rw, gyro_rw]` via
  `state.set_imu_params`, prior sigmas.
* **No initial alignment is provided**: gravity direction, scale, velocity and biases are estimated by a VINS-style visual-inertial
  initialisation (`init_VI`), retried every frame until the IMU has enough excitation (std of dv/dt over 8 keyframes >= 0.25). The world frame is
  gravity-aligned (z up), origin/yaw arbitrary -> evaluation needs SE(3) alignment (as CONTRACT says). Nothing is output before initialisation succeeds.
* Output: `resultpath` text file, one line `t x y z qx qy qz qw` per pose (seconds, **IMU/body pose** `wTb`, metric scale), written from several
  places (init block, per-keyframe, high-rate IMU propagation) -> may be unsorted/duplicated; `scripts/to_tum.py` sorts, keeps the last estimate per timestamp,
  drops non-finite rows, renormalises quaternions.
* Important caveat: upstream has **no EuRoC demo** (TUM-VI, KITTI-360, SubT, WHU only; it only ships `calib/euroc.txt` and `dataset/euroc_to_hdf5.py`).
  `configs/euroc.yaml` therefore uses the official EuRoC calibration/extrinsics but the *TUM-VI* algorithm parameters and IMU noise (same 20 Hz camera /
  200 Hz IMU). They are untuned; expect results possibly worse than a tuned run. Document this when comparing methods.

## 4. Layout

```
methods/dbafusion/
  dbafusion_colab.ipynb     main notebook (parameters at the top)
  configs/euroc.yaml        calibration, T_imu_cam, IMU noise, DBA-Fusion hyper-parameters
  configs/TEMPLATE_ground_robot.yaml   template + notes (excitation, units, extrinsics convention)
  scripts/asl_to_dbafusion.py   ASL -> DBA-Fusion inputs (+ sanity check CLI, GT csv -> TUM)
  scripts/run_dbafusion.py      driver (replaces demo_vio_tumvi.py; needs CUDA)
  scripts/to_tum.py             raw result -> TUM
  scripts/eval_traj.py          evo APE/RPE + plots
  scripts/patch_build.py, gtsam_cache.py, fetch_euroc.py, shims/torch_scatter.py
  tests/test_scripts.py         CPU tests on synthetic data
```

## 5. Step by step (user)

1. **ClearML**: create a free account at https://app.clear.ml -> Settings -> Workspace -> *Create new credentials*. Note the five values
   `CLEARML_API_ACCESS_KEY`, `CLEARML_API_SECRET_KEY`, `CLEARML_API_HOST=https://api.clear.ml`, `CLEARML_WEB_HOST=https://app.clear.ml`, `CLEARML_FILES_HOST=https://files.clear.ml`.
2. **Colab Secrets** (key icon in the left bar of Colab): add the five names above with those values, switch *Notebook access* ON for each.
3. **Data on Drive**: `MyDrive/vio-bench/data/<dataset>/<seq>/asl/mav0/{cam0,imu0}/...` and `MyDrive/vio-bench/data/<dataset>/<seq>/gt_tum.txt`
   (CONTRACT.md layout; copy from `data/` on the laptop). For the EuRoC sanity check you can instead set `DOWNLOAD_EUROC = True`
   (downloads MH_01_easy, ~1.5 GB zip, from the ETH mirror; if the URL has moved edit `scripts/fetch_euroc.py`/pass `--url`).
4. **Code**: after pushing the repo set `REPO_URL` in the first cell. Until then upload the folder `methods/dbafusion` to `/content/dbafusion`
   (Colab Files pane) or copy it to `MyDrive/vio-bench/code/dbafusion` (the notebook checks both before cloning).
5. Open `dbafusion_colab.ipynb` in Colab (File -> Upload notebook), Runtime -> Change runtime type -> GPU (T4 is enough; L4/A100 also supported), edit cell 0
   (`DATASET`, `SEQ`), **Run all**. First run is slow because of the builds (see below); keep the tab open. If the session dies, reconnect and Run all again:
   builds come from the Drive cache, finished runs are skipped.
6. Results: `MyDrive/vio-bench/results/dbafusion/<dataset>/<seq>/` with `traj_tum.txt` (contract format), `log.txt`, `run_stats.json`,
   `eval_metrics.json`, `config_*.yaml`, plots, `raw_result.txt`. ClearML project `vio-bench`, task `dbafusion/<dataset>/<seq>`: hyper-parameters (config, versions, GPU),
   scalars (APE_translation_m, RPE_translation_m, runtime incl. GPU peak memory), plots, artifacts (trajectory, metrics, config, log), plus ClearML's automatic GPU/CPU monitoring.
   Download `traj_tum.txt` to the laptop `results/dbafusion/...` and evaluate it with the shared `eval/` tools for apples-to-apples numbers.
7. Quick smoke test before the full run: `MAX_FRAMES = 100` (writes `*_smoke` files, never the final result).

### Expected runtime (estimates -- NOTHING here was run on a GPU)
* First session on a fresh Drive cache: apt+pip ~2-3 min; GTSAM build ~20-45 min (2 vCPU); lietorch+droid_backends nvcc build ~8-20 min.
* Later sessions: cache restore ~1-2 min.
* EuRoC MH_01_easy (3682 frames @20 Hz, stride 4 -> ~920 processed frames): I expect roughly 3-10 min on a T4 (upstream claims near-real-time on a desktop GPU); GPU memory ~10 GB at `max_factors=48` (T4 has 15 GB; use `OVERRIDES={"dbaf.max_factors": 36}` for ~6 GB).

## 6. Known issues / risks

* **Untested on GPU.** The build recipe is derived from reading the code, not from a successful Colab run. The most likely failure points, in order:
  (1) GTSAM fork (2023-era dev branch, Boost/pybind11) vs. Colab's current Python/compiler; (2) nvcc/Eigen/torch-API incompatibility when compiling
  `droid_kernels.cu` against a very new torch (patches here only cover `.type()` and arch flags); (3) nvcc major version != torch's CUDA major (torch refuses to build);
  (4) VI initialisation never succeeding on a sequence (see below).
* EuRoC parameters are borrowed from TUM-VI (no upstream EuRoC recipe); camera-IMU time offset is not estimated.
* Initialisation needs excitation; EuRoC MH_01 starts nearly static for a few seconds -> trajectory begins later than GT (evo associates on the overlap; `n_associated_poses` is logged).
  If it never initialises, the raw file stays empty and `to_tum.py` raises with a clear message.
* Online (causal) output: poses are the real-time estimates, not a smoothed post-hoc trajectory (upstream also evaluates this).
* `torch.load` of `droid.pth` is fine with `weights_only=True` (plain tensors); the checkpoint comes from the official DBA-Fusion/DROID-SLAM links via gdown.
* Google Drive FUSE I/O is slow, therefore the sequence is rsync'ed to `/content/work`.
* Timestamps are `ns/1e9` in float64 (about 0.2 us resolution), identical to upstream.
* Monocular: scale comes from the IMU, so RPE/APE use SE(3) alignment without scale as required; the logged `align_scale` is 1 by construction.
  A failed initialisation can still give scale-drifted output; check the plots.

## 7. Fallback plan if the build fails on current Colab

1. **Pin an older torch** in a fresh runtime (then Runtime -> Restart): e.g. `pip install torch==2.1.2 torchvision==0.16.2 --index-url https://download.pytorch.org/whl/cu121`
   (needs a CUDA 12.x toolkit on the VM; cache key changes automatically so wheels are rebuilt). Upstream's own pin (torch 1.11/cu113, py3.10) cannot be installed on Colab's python directly;
   if really needed, create a py3.10 venv with `uv venv --python 3.10` and run `run_dbafusion.py` from it (the driver is self-contained; build gtsam for that python).
2. **GTSAM fails**: try the fork's other commits (`a4e958f1`, `0364d8e7`), force the bundled pybind11, or build with `-DGTSAM_USE_SYSTEM_EIGEN=ON`.
3. **droid_backends fails**: read the nvcc error, usually an Eigen/CUDA or removed-torch-API issue; try `--eigen_dir /usr/include/eigen3` (done automatically) and an older torch (1).
4. **Give up on tight fusion**: run the visual-only baseline instead, clearly labelled as such: DBA-Fusion with `dbaf.visual_only=1` (still needs the build), or DPVO / DROID-SLAM
   (https://github.com/princeton-vl/DPVO has `evaluate_euroc.py`; same CUDA-extension build issues but no GTSAM). Visual-only is scale-ambiguous, so it would need Sim(3) alignment
   and would not be comparable under the contract's SE(3)-no-scale rule -- report it separately with that caveat.

## 8. Local tests

`python tests/test_scripts.py` (needs numpy, scipy, pyyaml, opencv, torch, evo, matplotlib): config validation, ASL readers/unit conversion, frame selection, image
preprocessing sizes, GT conversion, `to_tum` de-duplication, evo APE/RPE on a synthetic rigidly transformed trajectory, torch_scatter shim, build-patch idempotence.
The notebook JSON validates with nbformat and every code cell parses; CUDA, GTSAM, ClearML and Colab-specific cells are untested.

## 9. ROVER T265 (notebook v2: list of sequences, rectification in Colab)

* `configs/rover_t265.yaml` = `tools/rover/generated/dbafusion/rover_t265.yaml`: DBA-Fusion needs pinhole images, so the notebook runs `tools/rover/undistort_asl.py`
  on Colab's local disk (`/content/work/undist/<seq>`, 640x480, hfov 100 deg, R = I so `T_imu_cam` is unchanged; ~1 GB per sequence, never written back to Drive) before the
  adapter reads it. The notebook asserts that the config's `fx/width` match its `ROVER_RECT` constants.
* IMU: ROVER's `imu0/data.csv` is already EuRoC format (200 Hz, rad/s, m/s^2): `gyro_unit: rad_s`, the adapter converts to deg/s. Checked locally with
  `asl_to_dbafusion.py --check` on a rectified 15 s snippet: 200.3 Hz, accel norm 9.84, gyro max 82 deg/s, 113 frames at stride 4.
* Parameters at the top of the notebook: `SEQUENCES = [("rover", ...), ...]` (all run in one session, failures do not stop the loop), `CONFIG_BY_DATASET`, `OVERRIDES`, `MAX_FRAMES`, `RPE_DELTA_M = 10`.
* Code: `scripts/upload_drive.sh` puts `methods/dbafusion`, `tools`, `eval`, `CONTRACT.md` at `MyDrive/vio-bench/repo` (the notebook rsyncs it to local disk) and the data at
  `MyDrive/vio-bench/data/<dataset>/<seq>/{asl,gt_tum.txt}`.
* Evaluation in the notebook runs the repo's `eval/evaluate.py` (ROVER: `--trans-only --interp-est --lever-arm-calib tools/rover/calib/calib_t265.yaml`), so numbers are the same as on the laptop
  (`pixi run fetch-colab` re-runs the evaluation locally next to OpenVINS/ORB-SLAM3). DBA-Fusion outputs the **IMU/body** pose, hence the lever arm applies.
* **Untested** (no GPU here): everything that runs in Colab (builds, driver, ClearML, the rectification cell, the evaluation cell). Tested locally: notebook JSON (nbformat) and cell syntax,
  `undistort_asl.py` + `asl_to_dbafusion.py --check` on ROVER data, `eval/evaluate.py` with the exact notebook command line on another method's trajectory.
  Config risks: noise values are Kalibr x2 and untuned (the euroc/TUM-VI values are very different, e.g. gyro random walk 1.4e-4 vs 7e-3); VI initialisation needs excitation, ROVER starts stationary for ~10 s
  and the robot moves slowly (~0.4 m/s, planar), so initialisation may be late or fail; fixed `stride: 4` = 7.5 Hz keyframe candidates at 30 fps.
