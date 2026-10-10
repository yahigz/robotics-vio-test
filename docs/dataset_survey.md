# Ground-robot outdoor visual-inertial datasets: survey (2026-10-10)

Goal: VIO evaluation (OpenVINS, ORB-SLAM3 mono/stereo-inertial, DBA-Fusion) with evo APE/RPE.
Constraint: each sequence (or per-sensor subset: one camera + IMU + GT) must be downloadable at <= 5 GB.
All sizes below were measured by HEAD / Range requests or by reading remote ZIP central directories / HF tree APIs. Nothing large was downloaded.

## Ranked summary

| # | Dataset (year / venue) | Platform | Outdoor, <=5 GB per sensor-subset | Cam / IMU | GT | Calib | Host, links alive? | Format |
|---|---|---|---|---|---|---|---|---|
| 1 | **ROVER** (2025, arXiv 2412.02506, Esslingen Univ.) | Wheeled lawn-mower robot, 0.5 m/s | 39 sequences, 5 outdoor sites. T265 left cam + IMU = 2.9-4.6 GB per seq (full zips are 11-48 GB, so ranged extraction from ZIP is needed) | RealSense T265: 2x fisheye mono 848x800, **global shutter, 30 Hz**; IMU 200 Hz (T265 BMI055). Also D435i (RGB rolling shutter) + IMU, Pi cam (rolling), VN100 IMU 65 Hz | Leica TS16 total station, 3-DoF position, ~1 mm, 2-5 Hz (about 2.4 Hz in practice), `groundtruth.txt` TUM-like with **zero quaternions** | Kalibr intrinsics, T_imu_cam, T_cam_prism, IMU noise/walk (`calibration.zip`, 3 KB) | fdm.hs-esslingen.de (HTTP 200, `Accept-Ranges: bytes`, Range works, checked) and HuggingFace `iis-esslingen/ROVER` (200) | Raw files in ZIP: PNG + txt. ROS-bag generation scripts provided (rover_benchmark repo) |
| 2 | **GrandTour** (2026, arXiv 2602.18164, ETH RSL) | ANYmal-D quadruped (legged), Boxi payload | 49 missions (about 35 outdoor with GNSS). Per mission: one cam 0.1-2.3 GB + IMU 10 MB + GT 7 MB | Alphasense CoreResearch: **global shutter, 10 Hz**, 1440x1080, hw-triggered, PTP; Bosch BMI085 IMU **200 Hz** in the cam unit. Also HDR (rolling, 30 Hz), ZED2i (rolling, 15 Hz, IMU 45 Hz, unreliable stamps), STIM320 500 Hz | NovAtel CPT7 + Inertial Explorer tightly-coupled post-processed GNSS/INS 6-DoF about 200 Hz (`cpt7_ie_tc_tf`); Leica MS60 total-station prism position about 7 Hz (`prism_position`, partial coverage); paper's "Holistic Fusion" GT mean ATE 2.8 mm | Camera-info yaml (K, D) + `tf.yaml` (box_base->cam / imu). Kalibr cam-IMU, Allan-variance IMU noise (in `grand_tour_box` repo, not verified) | HuggingFace `leggedrobotics/grand_tour_dataset` (public, not gated, per-sensor tar, Range works, checked). ROS1 bags need registration (kleinkram) | Zarr (IMU/poses) inside tar; images are JPEG files inside tar |
| 3 | **FusionPortable V2** (2024/25, IJRR) | UGV (Mini-Hercules), Unitree legged, handheld, vehicle | `legged_grass01` 3.62 GB (355 s, outdoor, 97 m). `ugv_campus01` 5.60 GB (183 s, 343 m, slightly over). `legged_grass00` 10.1 GB, `ugv_campus00` 10.0 GB are over. `ugv_parking*` and `*_transition*` are underground / GNSS-denied | FLIR BFS-U3-31S4C stereo, **global shutter, 20 Hz**, 1024x768 (published as `/compressed`); STIM300 IMU **200 Hz** (also event cams, Ouster) | UGV and legged: Leica MS60 total station, 3-DoF 1 mm, resampled to 20 Hz, TUM-format txt. Vehicle: RTK-GNSS/INS | Intrinsics, T_cam_imu, timeshift, STIM300 noise: github.com/fusionportable/calibration_files (`20230618_calib` for UGV) | Google Drive (works through usercontent endpoint with Range) and Baidu. Whole bag only (no per-sensor split) as `.7z` | ROS1 bag in 7z, GT `.txt` separate |
| 4 | Rosario v1 (2019, IJRR) | Wheeled weed-removal robot, soybean field | seq03 1.83 GB (3.3 min), seq04 1.64 GB (2.7 min), seq02 2.80 GB (4.4 min), seq05 2.78 GB (5.2 min), seq06 3.92 GB, seq01 5.84 GB (9.3 min, over) | ZED stereo 672x376 **15 Hz, rolling shutter**, software-synced (ms stamps); LSM6DS0 IMU 140 Hz (low grade) | Two RTK-GPS modules at 5 Hz, position only (`sequenceXX_gt.txt`, identity quaternions), ~cm | `calibrationXX.yaml` (Kalibr-style T_cam_imu, no IMU noise block) | robot.cifasis-conicet.gov.ar -> fs01.cifasis-conicet.gov.ar:90 (plain HTTP, all HEAD 200). Chunks of 2 GB (`.bag.00`, `.01`) | ROS1 bags (compressed), raw tar.gz |
| 5 | Rosario v2 (2025, IJRR) | 4-wheel agri robot | Base bags 29-92 GB. Only GT / GNSS addon bags are small (4-11 MB) | RealSense D435i (IR stereo global shutter + RGB), IMUs, GNSS RTK / PPK, hw-sync | 6-DoF pose GT (`_pgt.bag` 3.4-28 GB, `mins_tum/*.csv` 1-4 MB in TUM format) | Kalibr yamls | fs01.cifasis-conicet.gov.ar:90 (alive) | Reject: no per-sensor bags, all > 29 GB |
| 6 | M2DGR-plus (2023/24) | Ground robot, SJTU | Street_01 1.7 GB, Bridge_01 2.4, Tree 3.7, Street_02 3.9, Parking_01 3.3, Building 3.4-3.7 | D435i RGB (rolling) 15 Hz, IMU 200 Hz | README lists no GT topic or file for outdoor runs (Xsens RTK and Vicon are mentioned as sensors only) | `calibration.txt` in repo | OneDrive/SharePoint; curl gets 302 then 403 (browser only) | Reject: GT not published, rolling shutter, links not scriptable |
| 7 | Ground-Challenge (2023) | Ground robot | 36 seqs, 1.2-9.9 GB each, but scenes are corridors, halls, offices, darkroom (indoor). `Roughroad3` is 1.8 GB / 59 s | D435i 15 Hz, IMU 200 Hz | Vicon indoor only | in repo | OneDrive | Reject: indoor |
| 8 | i2Nav-Robot (2025) | Wheeled AgileX Ranger Mini 3, campus | 10 seqs, total about 450 GB (individual bags tens of GB) | AVT Mako stereo global shutter but **10 Hz**; ADIS16465 200 Hz | Navigation-grade INS + GNSS PPK, LiDAR map matching, 6-DoF TUM csv | yes | Baidu / OneDrive | Reject: 10 Hz, huge bags, Baidu / OneDrive |
| 9 | SMARTNav (2025, Saxion) | Ground (greenhouse, sidewalk) + air | `greenhouse_ground_1/2` 7.3 / 7.7 GB; `sidewalk_ground_1` 35.9 GB; all > 5 GB | ZED-X stereo + IMU | RTK or GLIM LiDAR-SLAM | yes | Saxion SURF drive | Reject: ground bags > 5 GB; greenhouse is not true outdoor. Small ones are flights / handheld |
| 10 | M-SEVIQ (2026) | Unitree Go2 quadruped | 40+ outdoor seqs | D435i 1280x720 (RGB rolling), event stereo | RTK-GPS position | claimed | anonymous.4open.science mirror only | Not usable: no stable public host, sizes unknown |
| 11 | CODa (UT Austin, 2023) | Jackal | 22 seqs, full 4 TB | cams 10 Hz, IMU **40 Hz** | LiDAR-SLAM pseudo-GT | yes | TACC / TDR | Reject: rates too low |
| 12 | MCD (CVPR 2024), Kimera-Multi, TartanDrive, Newer College | handheld/ATV, Jackal fleet (data on request), ATV, handheld | -- | -- | -- | -- | -- | Reject: handheld/ATV, access on request, or sizes unverified (searched only, not deep-checked) |

## Top-1 recommendation: ROVER (strictly meets every stated requirement), GrandTour as the easy backup

### Why ROVER

- Wheeled UGV, outdoor, 30 Hz global-shutter fisheye + 200 Hz IMU, 1 mm total-station GT, published Kalibr calibration with IMU noise, working HTTP server with Range support.
- Cost: sequences are shipped as one big ZIP each (11-48 GB). Inside each ZIP the images are PNG (compression method 8, already incompressible), stored contiguously per folder. I verified from the central directory that `realsense_T265/cam_left/*.png` is one contiguous byte range, and verified that a ranged fetch plus raw-deflate decode of `groundtruth.txt` and `cam_left.txt` works. So you can pull only the T265-left folder + IMU + GT.
- Size of the mono-inertial subset (T265 cam_left + imu.txt + groundtruth + cam_left.txt):

| Sequence (zip name) | Conditions | Duration | T265 cam_left PNGs | IMU txt | Total subset | Full zip |
|---|---|---|---|---|---|---|
| `garden_small_2024-05-29_1` (day) | small shaded garden, 100 sqm | 327 s (9809 frames) | 2.87 GB | 4 MB | **2.9 GB** | 17.0 GB |
| `campus_small_2024-02-19` (winter) | lawn beside building | 395 s (11836) | 3.52 GB | 5 MB | **3.5 GB** | 19.7 GB |
| `park_2023-07-31` (summer) | natural park, slopes, trees | 438 s (13139) | 4.17 GB | 6 MB | **4.2 GB** | 25.8 GB |

Other small subsets: garden_small_2023-08-18 3.3 GB, garden_small_2024-05-29_3 (night) 3.05 GB, garden_large_2024-05-29_1 3.2 GB, campus_small_2024-05-07 3.7 GB. T265 right camera adds about the same again, so stereo-inertial with full length is about 5.8 GB (over the limit); use a time-trimmed prefix (contiguous range) for stereo.

### URLs (all `https://fdm.hs-esslingen.de/schmidt2025rover/...`, HTTP 200, Accept-Ranges: bytes)

- `calibration.zip` (3 KB): `calib_t265.yaml`, `calib_d435i.yaml`, `calib_picam.yaml`, `calib_vn100.yaml`
- `garden_small_2024-05-29_1.zip` (16,999,736,598 bytes)
- `campus_small_2024-02-19.zip` (19,686,103,501 bytes)
- `park_2023-07-31.zip` (25,794,849,763 bytes)
- Mirror: https://huggingface.co/datasets/iis-esslingen/ROVER (chunked files; reassembly script https://github.com/iis-esslingen/rover_benchmark/blob/main/utils/download.py)
- ROS bag generation scripts: rover_benchmark repo (linked from https://iis-esslingen.github.io/rover/pages/dataset_overview)

### Layout inside each zip (top folder = sequence name with date, e.g. `2024-05-29_1/`)

```
groundtruth.txt                       # t x y z 0 0 0 0   (781 rows for the 327 s sequence, ~2.4 Hz)
realsense_T265/cam_left/<t>.png       # 848x800 mono8 fisheye, t = unix time with decimals, 30 Hz
realsense_T265/cam_left.txt           # "<t> rgb/<t>.png"   (note: lists rgb/ prefix, actual folder is cam_left/)
realsense_T265/cam_right/ , cam_right.txt
realsense_T265/imu/imu.txt            # 4 MB for 327 s
realsense_D435i/{rgb,depth,imu}, pi_cam/rgb, vn100/imu.txt
```

### Extraction without downloading the zip (Range requests)

Parse the ZIP64 central directory with 2 Range requests (last 70 KB, then the directory), pick entries starting with `.../realsense_T265/cam_left/`, `.../realsense_T265/imu/`, `groundtruth.txt`, `cam_left.txt`; fetch each local header (30 bytes + name + extra) then its data and `zlib.decompress(data, -15)`. For bulk, fetch the single contiguous range from the first to the last cam_left entry's offset, then walk it. The `remotezip` pip package does the same. Note: this host's TLS chain failed verification in Python on macOS (curl was fine); use `curl --cacert`/system store or `requests` with certifi, do not disable verification for the real download.

### ROVER gotchas

- GT is **position only**, ~2.4 Hz, and the quaternion columns are `0 0 0 0` (invalid). Convert to TUM with identity quaternion and use `evo_ape tum --align` (SE3 or Sim3) and translation-only RPE; do not use orientation metrics. First GT stamp is about 1 s before the first image.
- GT point is the prism, not the IMU. `calib_t265.yaml` gives `CamLeft-To-Prism` and `IMU-To-CamLeft`; with `--align` this lever arm (about 0.40 m forward, -0.19 m) matters slightly for APE; apply it if you want exact numbers.
- T265 fisheye uses the **equidistant** model (4 coeffs k1-k4: -0.01153, 0.05015, -0.05041, 0.01274; fx,fy,cx,cy = 280.44, 279.58, 434.59, 395.37). OpenVINS: `equidistant`. ORB-SLAM3: `KannalaBrandt8`. DBA-Fusion: check its camera model support.
- IMU noise in calib: gyro 3.24e-3, acc 1.74e-2, gyro walk 7.06e-5, acc walk 1.24e-2 (these are the estimated Kalibr-style values; consider inflating).
- T265 IMU: accel about 62.5 Hz and gyro 200 Hz are published separately ("65, 200 Hz" in the paper); `imu.txt` is already merged, but the bag generation script interpolates. Check the actual rate in `imu.txt` before trusting 200 Hz.
- Clocks: Unix epoch seconds (NTP / GNSS-synced via a Raspberry Pi NTP server); images, IMU and GT share the same clock. No hw trigger between cameras and IMU: estimate td online (OpenVINS `calib_camimu_dt`).
- Platform is vibration-prone (0.5 m/s, rough lawn); T265 is fisheye with a large FoV and is mono8; sequences are seasonal repeats of the same loops (good for loop-closure tests).
- ROS-bag format is not provided, only raw files (scripts convert). No compressed-image issue: PNG lossless.

### Backup: GrandTour (easiest download, 6-DoF GT)

Per-mission files are individually addressable on HuggingFace:
`https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset/resolve/main/<MISSION>/{images,data,metadata}/...`

Recommended missions (all outdoor, GNSS working, Train split):

| Short | Folder | Dur. | Scene | alphasense_front_left.tar | alphasense_front_right.tar |
|---|---|---|---|---|---|
| ETH-1 | `2024-10-01-11-29-55` | 333 s | pavement, people, cars | 0.62 GB (619,765,760 B) | 0.74 GB (740,771,840 B) |
| GRI-1 | `2024-11-04-10-57-34` | 455 s | urban, grass, pavement | 0.89 GB (887,941,120 B) | about 0.9 GB |
| CYN-1 | `2024-11-04-12-55-59` | 510 s | forest, mountain trail | 1.40 GB (1,400,320,000 B) | about 1.4 GB |

Other options: SNOW-1 (`2024-11-03-07-52-45`, 172 s, 0.42 GB), PIL-1 (`2024-11-11-12-07-40`, 946 s, 2.25 GB). For each mission also fetch:
- `data/alphasense_imu.tar` (about 10 MB; Zarr; `ang_vel`, `lin_acc`, `timestamp`; 37,002 samples in 172 s i.e. about 215 Hz incl. extras, BMI085)
- `data/cpt7_ie_tc_tf.tar` (about 7 MB; Zarr `translation`, `rotation`, `timestamp`; 37,201 poses / 172 s, about 200 Hz) = ground truth, pose of `cpt7_imu` frame
- `data/prism_position.tar` (3 MB; 1201 points / 172 s, Leica MS60, partial coverage)
- `metadata/tf.yaml`, `metadata/alphasense_front_left_caminfo.yaml`, `metadata/alphasense_front_right_caminfo.yaml`, `metadata/alphasense_imu.yaml`, `metadata/cpt7_ie_tc_tf.yaml`
Images are `NNNNNN.jpeg` (about 376 KB each) inside `alphasense_front_left/` in the tar; timestamps come from the sibling Zarr/yaml (not verified: look at `explore.ipynb` in github.com/leggedrobotics/grand_tour_dataset/examples_hugging_face).

GrandTour gotchas:
- Camera is **10 Hz** (below your 15 Hz wish) and platform is a **quadruped** (gait vibration, pitch/roll oscillation). Frame-rate is acceptable for OpenVINS and ORB-SLAM3 but expect worse tracking than at 20-30 Hz.
- No ready T_cam_imu: compose `box_base->alphasense_front_left` and `box_base->alphasense_imu` from `tf.yaml` (T_cam_imu = inv(T_base_cam) * T_base_imu). Camera model is pinhole + 4-coef plumb-bob (undistorted K given in `P` identical to `K`). IMU noise parameters not found in the HF metadata.
- GT is for `cpt7_imu` (or the prism); the CPT7 IMU frame and the Alphasense IMU are different frames: apply the static transform from tf.yaml before RPE if you use the IMU/camera pose. `evo --align` absorbs the world frame but not the lever arm.
- Some train missions have no or partial GNSS (ETH-2, HAUS-1, ARC-6, CON-*): check the `gnss` tag in https://grand-tour.leggedrobotics.com/data/missions.json. Test-split missions (EIG-1, SNOW-2, SPX-2, CON-4, ARC-2, ARC-7) have redacted GT for the COMFORT benchmark.
- Early missions (ETH-1/2/3) have imprecise HDR timestamps only (the Alphasense cameras are not affected). ZED2i stamps are unreliable; avoid.
- The ZARR tars use blosc/lz4 compression: install `zarr<3` + `numcodecs`.

### Third option: FusionPortableV2 `legged_grass01`

- File: `legged_grass01.7z`, Google Drive id `1KZXxai2ZrA5W9of4PbVpUiDQgVpvn82M`, 3.62 GB (content-range check: `curl -s -D - -o /dev/null -H "Range: bytes=0-0" "https://drive.usercontent.google.com/download?id=<ID>&export=download&confirm=t"`). Drive folder: https://drive.google.com/drive/folders/1PYhnf3PlY5r0hbyzWDGTUTPxRMl6SYa-
- GT: `groundtruth/traj/legged_grass01.txt` id `1yG5K_yJDQUvpb947LJumvwiDOUKm93-H` (TUM). Calibration: `20230618_calib` for UGV, `20230912_calib` for legged (check README of the calibration repo).
- Topics (from calibration yaml): `/stereo/frame_left/image_raw/compressed` (JPEG, 20 Hz), `/stim300/imu/data_raw` (200 Hz). Body-IMU noise: acc density 1.14e-3, gyro density 6.08e-5.
- Gotchas: sequence is small (97 m, 355 s on grass); legged calib file has TODO placeholders for the Unitree IMU only (not needed). Frame-cam to body_imu `timeshift = -0.0136 s`. GT is 3-DoF total station resampled by cubic spline (orientation not measured).
- `ugv_campus01` (5.60 GB, 183 s) is the only UGV outdoor run close to the limit; `ugv_campus00` is 10.05 GB.

## Verification notes

- ROVER server has no checksums page; ZIP central directories were read via Range for all 39 sequences (cam/IMU sizes above come from those).
- Rosario v1: ZED stereo is rolling shutter, sync is software only (paper), 15 Hz.
- Google Drive HEAD returns `content-length: 0`; use the Range trick above for sizes.
- Not deeply verified: MCD, TartanDrive, Kimera-Multi, M-SEVIQ details (rates/sizes), GrandTour IMU noise parameters, GrandTour world frame of `cpt7_ie_tc_tf`.

## ROVER: prepared data

Prepared by `tools/rover/` (T265 left fisheye + IMU, mono-inertial). Layout per `CONTRACT.md`: `data/rover/<seq>/asl/mav0/{cam0,imu0}`, `gt_tum.txt`, `manifest.json`, `preview/*.png`. Calibration: `data/rover/calib_t265.yaml` (copy in `tools/rover/calib/`). Method configs are generated into `tools/rover/generated/{openvins,orbslam3,dbafusion}/` by `tools/rover/make_configs.py` (`--noise-scale`/`--walk-scale`, default x2 over the Kalibr values).

| Sequence | Frames (30 Hz) | Duration | Size on disk | GT (prism) | Notes |
|---|---|---|---|---|---|
| `garden_small_2024-05-29_1` | 9778 (30 tail frames without IMU dropped) | 325.9 s | 2.7 GB | 781 pts, 2.4 Hz, 121 m | complete, motion starts about 10 s in |
| `campus_small_2024-02-19` | 6188 | 206 s (prefix of 395 s) | 1.8 GB | 290 pts, 1.4 Hz, 77 m | PREFIX only (slow network), motion starts about 10 s in |
| `park_2023-07-31` | 1800 | 60 s (prefix of 438 s) | 0.55 GB | 129 pts, 2.0 Hz, 19 m | PREFIX only, very short |

Mono only: stereo (left+right) would be 5.8 / 7.1 / 8.4 GB, over the 5 GB limit. Download is from the HuggingFace mirror (`fetch_subset.py`, ZIP Range requests, CRC32 verified); `fdm.hs-esslingen.de` was about 10x slower. Prefixes can be extended by re-running `fetch_subset.py fetch <seq> --max-seconds N` (resumes) but `convert_asl.py` deletes `raw/`, so extending means refetching.

Facts verified on the data:
- `imu.txt` columns are `t, ax, ay, az, gx, gy, gz` (m/s^2, rad/s); +g appears on +y. It is a merged 62.5 Hz accel + 200 Hz gyro stream (about 264 rows/s, other channel already interpolated). Rows closer than 2.5 ms to the previous kept row are dropped, leaving 200.3 Hz with no gaps (max dt 7.5 ms).
- Camera: exactly 30 Hz, no dropped frames (max dt 33.7 ms). `cam_left.txt` stamps are rounded; PNG file names (full precision) are used.
- One clock (unix seconds) for camera, IMU and GT. GT starts about 1 s before the first image (garden, park) or 0.35 s after (campus) and ends 0.2-1.9 s before the last image; max GT gap 0.8-1.4 s.
- GT is prism position only; `gt_tum.txt` has identity quaternions, so only translation metrics (APE trans, RPE trans) are meaningful. The prism is about 0.45 m from the IMU (position in IMU frame about (0.07, 0.17, -0.41) m, see `calib_t265.yaml: prism`); with SE(3)/Sim(3) alignment this lever arm adds a small bias that is not corrected (no orientation GT). The sign convention of `CamLeft-To-Prism` is inferred (unverified).
- Calibration: `IMU-To-CamLeft/Right` are Kalibr `T_cam_imu` and `CamRight-To-CamLeft` is `T_cam1_cam0` (checked to 1e-15). Equidistant intrinsics cam0: fx 280.44, fy 279.58, cx 434.59, cy 395.37, k = (-0.01153, 0.05015, -0.05041, 0.01274), 848x800. IMU (Kalibr, before inflation): gyro 3.24e-3, acc 1.74e-2, gyro walk 7.06e-5, acc walk 1.24e-2.
- ROS bag: not kept (about 816 MB/min with lz4, about 4.4 GB for the garden sequence). `tools/rover/asl_to_bag.py <asl> <tmp.bag> --duration 30` creates it on demand. DBA-Fusion needs pinhole images: `tools/rover/undistort_asl.py data/rover/<seq>` (640x480, hfov 100, fx 268.5, about 110 KB/frame) on demand, with a 5 GB budget check.
