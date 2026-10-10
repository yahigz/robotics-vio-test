"""CPU-only tests of the helper scripts on synthetic data (no CUDA / DBA-Fusion needed).
Run: python tests/test_scripts.py   (needs numpy scipy pyyaml opencv torch evo matplotlib)"""
import os, sys, tempfile, py_compile, importlib
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, os.path.join(ROOT, "scripts", "shims"))

import asl_to_dbafusion as A
import to_tum
import eval_traj


def make_seq(d, n_cam=60, cam_hz=20, imu_hz=200):
    import cv2
    mav = os.path.join(d, "asl", "mav0")
    os.makedirs(os.path.join(mav, "cam0", "data")); os.makedirs(os.path.join(mav, "imu0"))
    os.makedirs(os.path.join(mav, "state_groundtruth_estimate0"))
    t0 = 1403636579_000000000
    with open(os.path.join(mav, "cam0", "data.csv"), "w") as f:
        f.write("#timestamp [ns],filename\n")
        for i in range(n_cam):
            ts = t0 + int(i * 1e9 / cam_hz)
            f.write(f"{ts},{ts}.png\n")
            cv2.imwrite(os.path.join(mav, "cam0", "data", f"{ts}.png"),
                        (np.random.rand(480, 752) * 255).astype(np.uint8))
    n_imu = int((n_cam / cam_hz + 1.0) * imu_hz)
    with open(os.path.join(mav, "imu0", "data.csv"), "w") as f:
        f.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        for i in range(n_imu):
            ts = t0 - int(0.2e9) + int(i * 1e9 / imu_hz)
            f.write(f"{ts},0.1,-0.2,0.3,0.0,0.1,9.81\n")
    with open(os.path.join(mav, "state_groundtruth_estimate0", "data.csv"), "w") as f:
        f.write("#t,px,py,pz,qw,qx,qy,qz,vx,vy,vz,bw,ba\n")
        f.write(f"{t0},1,2,3,0.5,0.1,0.2,0.3,0,0,0,0,0,0,0,0\n")
    return os.path.join(d, "asl"), t0


def test_config():
    for name in ("euroc.yaml", "TEMPLATE_ground_robot.yaml"):
        cfg = A.load_config(os.path.join(ROOT, "configs", name))
        assert A.T_imu_cam(cfg).shape == (4, 4)
    cfg = A.load_config(os.path.join(ROOT, "configs", "euroc.yaml"), {"dbaf.max_factors": 36})
    assert cfg["dbaf"]["max_factors"] == 36
    # euroc T_BS must be a rigid transform
    T = A.T_imu_cam(cfg); assert abs(np.linalg.det(T[:3, :3]) - 1) < 1e-6


def test_adapter():
    cfg = A.load_config(os.path.join(ROOT, "configs", "euroc.yaml"))
    with tempfile.TemporaryDirectory() as d:
        asl, t0 = make_seq(d)
        mav = os.path.join(asl, "mav0")
        imu = A.read_imu_csv(os.path.join(mav, "imu0", "data.csv"), "rad_s")
        assert imu.shape[1] == 7 and abs(imu[0, 1] - np.degrees(0.1)) < 1e-9   # deg/s
        assert abs(imu[1, 0] - imu[0, 0] - 0.005) < 1e-6
        cam_t, names = A.read_cam_csv(os.path.join(mav, "cam0", "data.csv"))
        assert len(cam_t) == 60 and names[0].endswith(".png")
        assert not any("WARNING" in m for m in A.sanity_report(imu, cam_t, cfg))
        idx = A.select_frames(cam_t, imu[:, 0], 4)
        assert list(idx[:3]) == [0, 4, 8]
        idx2 = A.select_frames(cam_t, imu[:, 0], 4, max_frames=5)
        assert len(idx2) == 5
        gen = A.image_stream(os.path.join(mav, "cam0", "data"), cam_t, names, idx2, cfg)
        t, im, intr = next(gen)
        assert im.shape[0] == 1 and im.shape[1] == 3 and im.shape[2] % 8 == 0 and im.shape[3] % 8 == 0
        h1 = int(480 * np.sqrt(384 * 512 / (480 * 752)))
        assert im.shape[2] == h1 - h1 % 8, im.shape
        assert abs(intr[0].item() - 458.654 * (im.shape[3] / 752) * 1.0) < 458.654 * 0.02
        assert im.dtype.is_floating_point is False
        n = A.gt_csv_to_tum(os.path.join(mav, "state_groundtruth_estimate0", "data.csv"), os.path.join(d, "gt.txt"))
        v = np.loadtxt(os.path.join(d, "gt.txt"))
        assert n == 1 and np.allclose(v[4:], [0.1, 0.2, 0.3, 0.5])      # qx qy qz qw


def test_to_tum():
    with tempfile.TemporaryDirectory() as d:
        raw = os.path.join(d, "raw.txt"); out = os.path.join(d, "o.txt")
        lines = []
        for i in [3, 1, 2, 2, 0, 4, 5, 6, 7, 8, 9, 10]:
            lines.append("%.6f %.6f 0 0 0 0 0 2.0" % (i * 0.1, float(i) + (0.5 if i == 2 and len(lines) == 2 else 0)))
        lines.append("garbage line"); lines.append("1.0 nan 0 0 0 0 0 1")
        open(raw, "w").write("\n".join(lines) + "\n")
        r = to_tum.convert(raw, out)
        a = np.loadtxt(out)
        assert r["n_out"] == 11 and np.all(np.diff(a[:, 0]) > 0)
        assert abs(np.linalg.norm(a[0, 4:]) - 1) < 1e-9
        # duplicate at t=0.2: first occurrence has x=2.5, second x=2.0 -> last wins
        assert a[2, 1] == 2.0


def test_eval():
    from scipy.spatial.transform import Rotation as R
    with tempfile.TemporaryDirectory() as d:
        t = np.arange(0, 40, 0.05)
        p = np.stack([5 * np.cos(t / 4), 5 * np.sin(t / 4), 0.1 * t], 1)
        q = R.from_rotvec(np.stack([0*t, 0*t, t / 4], 1)).as_quat()
        gt = np.hstack([t[:, None], p, q])
        Rw = R.from_euler("xyz", [0.1, -0.2, 0.7]).as_matrix()
        pe = (Rw @ p.T).T + np.array([3, -1, 2]) + np.random.randn(*p.shape) * 0.01
        qe = (R.from_matrix(Rw) * R.from_quat(q)).as_quat()
        est = np.hstack([t[:, None] + 0.001, pe, qe])
        np.savetxt(os.path.join(d, "gt.txt"), gt, fmt="%.9f"); np.savetxt(os.path.join(d, "est.txt"), est, fmt="%.9f")
        res, figs = eval_traj.evaluate(os.path.join(d, "gt.txt"), os.path.join(d, "est.txt"), rpe_delta_m=1.0)
        assert res["ape_trans_m"]["rmse"] < 0.03, res
        assert abs(res["align_scale"] - 1.0) < 1e-9
        assert res["rpe_trans_m"]["rmse"] < 0.05, res
        assert set(figs) == {"trajectory", "ape", "rpe"}


def test_shim():
    import torch
    import torch_scatter as ts
    src = torch.randn(2, 7, 3); idx = torch.tensor([0, 2, 2, 1, 0, 4, 4])
    out = ts.scatter_sum(src, idx, dim=1, dim_size=6)
    ref = torch.zeros(2, 6, 3)
    for i, j in enumerate(idx): ref[:, j] += src[:, i]
    assert torch.allclose(out, ref, atol=1e-6)
    m = ts.scatter_mean(src, idx, dim=1)
    assert m.shape[1] == 5 and torch.allclose(m[:, 0], (src[:, 0] + src[:, 4]) / 2, atol=1e-6)
    assert torch.allclose(m[:, 3], torch.zeros(2, 3))
    e = ts.scatter_sum(torch.zeros(1, 0, 4), torch.zeros(0, dtype=torch.long), dim=1, dim_size=3)
    assert e.shape == (1, 3, 4)


def test_patch_build():
    import subprocess
    with tempfile.TemporaryDirectory() as d:
        os.makedirs(os.path.join(d, "src"))
        open(os.path.join(d, "src", "correlation_kernels.cu"), "w").write('AT_DISPATCH_FLOATING_TYPES_AND_HALF(volume.type(), "x", ([&] {}));')
        open(os.path.join(d, "src", "altcorr_kernel.cu"), "w").write('AT_DISPATCH_FLOATING_TYPES_AND_HALF(fmap1.type(), "y", ([&] {}));')
        for _ in range(2):   # idempotent
            subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "patch_build.py"), "--root", d, "--arch", "8.9"], check=True)
        assert ".scalar_type()" in open(os.path.join(d, "src", "altcorr_kernel.cu")).read()
        for f in ("setup_droid_backends.py", "setup_lietorch.py"):
            py_compile.compile(os.path.join(d, f), doraise=True)
        assert "compute_89" in open(os.path.join(d, "setup_lietorch.py")).read()


def test_compile_all():
    for f in os.listdir(os.path.join(ROOT, "scripts")):
        if f.endswith(".py"):
            py_compile.compile(os.path.join(ROOT, "scripts", f), doraise=True)


if __name__ == "__main__":
    for k, v in list(globals().items()):
        if k.startswith("test_"):
            v(); print("ok", k)
