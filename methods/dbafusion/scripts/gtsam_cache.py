"""Pack / unpack a built custom-GTSAM (python wrapper + shared libs) so later Colab sessions
skip the 20-40 min build. After `make python-install` the runtime needs:
  * <site-packages>/gtsam (+ its dist-info/egg-info),
  * /usr/local/lib/libgtsam*.so*, libmetis*.so*  (default ld path on Ubuntu).
Usage:  gtsam_cache.py pack OUT.tar.gz | unpack IN.tar.gz
"""
import glob
import os
import subprocess
import sys
import tarfile


def _files():
    import importlib.util
    spec = importlib.util.find_spec("gtsam")
    if spec is None:
        raise SystemExit("gtsam is not installed in this interpreter")
    pkg = os.path.dirname(spec.origin)
    site = os.path.dirname(pkg)
    out = []
    for root in (pkg,):
        for dp, _, fs in os.walk(root):
            out += [os.path.join(dp, f) for f in fs]
    for pat in ("gtsam-*", "gtsam_*"):                       # dist-info / egg-info / egg
        for p in glob.glob(os.path.join(site, pat)):
            if os.path.isdir(p):
                for dp, _, fs in os.walk(p):
                    out += [os.path.join(dp, f) for f in fs]
            else:
                out.append(p)
    for pat in ("libgtsam*.so*", "libmetis*.so*", "libCppUnitLite*"):
        out += glob.glob("/usr/local/lib/" + pat)
    return sorted(set(out))


def pack(dst):
    files = _files()
    with tarfile.open(dst, "w:gz") as t:
        for f in files:
            t.add(f, arcname=f.lstrip("/"))
    print(f"packed {len(files)} files -> {dst}")


def unpack(src):
    with tarfile.open(src) as t:
        t.extractall("/")
    subprocess.run(["ldconfig"], check=False)
    print("unpacked", src)


if __name__ == "__main__":
    cmd, path = sys.argv[1], sys.argv[2]
    {"pack": pack, "unpack": unpack}[cmd](path)
