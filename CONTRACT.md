# Interface contract (shared by all components)

Methods (one per branch of VIO history):
- filter-based:      OpenVINS (MSCKF, 2020)      -> methods/openvins   (local Docker, arm64, ROS1 Noetic)
- optimization-based: ORB-SLAM3 (2021)           -> methods/orbslam3   (local Docker, arm64)
- learned hybrid:    DBA-Fusion (RA-L 2024)      -> methods/dbafusion  (Google Colab GPU + ClearML)

Data layout (data/ is gitignored, each sequence <= 5 GB on disk):
- data/<dataset>/<seq>/raw/...            original download
- data/<dataset>/<seq>/<seq>.bag          ROS1 bag with camera + IMU topics only
- data/<dataset>/<seq>/asl/mav0/cam0/data/<ns>.png, cam0/data.csv, imu0/data.csv   (EuRoC/ASL format)
- data/<dataset>/<seq>/gt_tum.txt         ground truth, TUM format, timestamps in seconds

Results:
- results/<method>/<dataset>/<seq>/traj_tum.txt   "t tx ty tz qx qy qz qw", t in seconds (same clock as GT), pose of IMU/body frame in the method's world frame
- results/<method>/<dataset>/<seq>/log.txt

Sanity-check dataset: EuRoC MH_01_easy (data/euroc/MH_01_easy/...), GT converted to gt_tum.txt.
Evaluation: pixi tasks in root pixi.toml call eval/*.py (evo APE/RPE, SE(3) alignment, RPE delta in meters).
Docker: each method has methods/<m>/compose.yml; root docker-compose.yml uses `include:`.
