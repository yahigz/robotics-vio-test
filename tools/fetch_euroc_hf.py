#!/usr/bin/env python3
"""Fetch ONE EuRoC sequence (ASL zip) out of the HuggingFace mirror GlowBond/EuRoC_MAV_Dataset without downloading the whole group zip.

The mirror holds machine_hall.zip (12.7 GB), vicon_room1.zip, ... ; each contains <SEQ>.zip (stored or deflated) members.
We read the ZIP64 central directory with HTTP Range requests (curl), then pull only the byte range of <SEQ>.zip, verify its CRC32,
unzip it into <data_root>/euroc/<SEQ>/asl and convert the Leica/Vicon ground truth to TUM (tools/gt_to_tum.py logic).
Idempotent: skipped if asl/mav0/cam0/data.csv and gt_tum.txt exist. The raw zip is deleted afterwards (unless --keep-raw).

Usage: python tools/fetch_euroc_hf.py --seq MH_01_easy --data-root /content/data [--group machine_hall]
Needs: curl, unzip, numpy.
"""
import argparse, os, shutil, struct, subprocess, sys, zlib

HF = "https://huggingface.co/datasets/GlowBond/EuRoC_MAV_Dataset/resolve/main"


def curl(url, out=None, rng=None, tries=6):
    cmd = ["curl", "-sSfL", "--connect-timeout", "30", "--retry", "3", "--speed-limit", "5000", "--speed-time", "60"]
    if rng: cmd += ["-r", f"{rng[0]}-{rng[1]}"]
    if out: cmd += ["-o", out]
    for k in range(tries):
        r = subprocess.run(cmd + [url], capture_output=True)
        if r.returncode == 0:
            return r.stdout
        time_sleep(2 * (k + 1))
    raise RuntimeError(f"curl failed: {r.stderr[-300:]!r}")


def time_sleep(s):
    import time; time.sleep(s)


def remote_size(url):
    r = subprocess.run(["curl", "-sSfLI", url], capture_output=True, text=True, check=True)
    n = [l.split(":")[1].strip() for l in r.stdout.splitlines() if l.lower().startswith("content-length")]
    return int(n[-1])


def central_directory(url, size):
    tail_len = min(size, 70000)
    tail = curl(url, rng=(size - tail_len, size - 1))
    i = tail.rfind(b"PK\x05\x06")
    (_, _, _, nent, _, cd_size, cd_off, _) = struct.unpack("<IHHHHIIH", tail[i:i + 22])
    if cd_off == 0xFFFFFFFF or nent == 0xFFFF or cd_size == 0xFFFFFFFF:
        j = tail.rfind(b"PK\x06\x07")
        (_, _, eocd64_off, _) = struct.unpack("<IIQI", tail[j:j + 20])
        e = curl(url, rng=(eocd64_off, eocd64_off + 55))
        (_, _, _, _, _, _, nent, _, cd_size, cd_off) = struct.unpack("<IQHHIIQQQQ", e)
    cd = curl(url, rng=(cd_off, cd_off + cd_size - 1))
    ents, p = [], 0
    while p < len(cd):
        (sig, _, _, _, meth, _, _, crc, csz, usz, nl, el, cl, _, _, _, lho) = struct.unpack("<IHHHHHHIIIHHHHHII", cd[p:p + 46])
        assert sig == 0x02014B50
        name = cd[p + 46:p + 46 + nl].decode("utf-8", "replace")
        ex = cd[p + 46 + nl:p + 46 + nl + el]
        q = 0
        while q + 4 <= len(ex):
            hid, hsz = struct.unpack("<HH", ex[q:q + 4])
            if hid == 1:
                d, k = ex[q + 4:q + 4 + hsz], 0
                if usz == 0xFFFFFFFF: usz = struct.unpack("<Q", d[k:k + 8])[0]; k += 8
                if csz == 0xFFFFFFFF: csz = struct.unpack("<Q", d[k:k + 8])[0]; k += 8
                if lho == 0xFFFFFFFF: lho = struct.unpack("<Q", d[k:k + 8])[0]; k += 8
            q += 4 + hsz
        ents.append(dict(name=name, method=meth, crc=crc, csize=csz, usize=usz, off=lho))
        p += 46 + nl + el + cl
    assert len(ents) == nent, (len(ents), nent)
    return ents


def crc_file(path):
    c = 0
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 24), b""): c = zlib.crc32(b, c)
    return c & 0xFFFFFFFF


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seq", default="MH_01_easy"); ap.add_argument("--data-root", required=True)
    ap.add_argument("--group", default=None, help="machine_hall | vicon_room1 | vicon_room2 (default from the sequence name)")
    ap.add_argument("--keep-raw", action="store_true")
    ap.add_argument("--url", default=None, help="override the group zip URL (testing; file:// works)")
    a = ap.parse_args()
    group = a.group or ("machine_hall" if a.seq.startswith("MH") else "vicon_room1" if a.seq.startswith("V1") else "vicon_room2")
    base = os.path.join(a.data_root, "euroc", a.seq); asl = os.path.join(base, "asl"); gt = os.path.join(base, "gt_tum.txt")
    if os.path.isfile(os.path.join(asl, "mav0", "cam0", "data.csv")) and os.path.isfile(gt):
        print("already present:", base); return
    url = a.url or f"{HF}/{group}.zip"
    size = remote_size(url)
    ents = central_directory(url, size)
    want = [e for e in ents if os.path.basename(e["name"]) == a.seq + ".zip"]
    assert want, f"{a.seq}.zip not in {group}.zip; members: {[e['name'] for e in ents][:20]}"
    e = want[0]
    print(f"member {e['name']}: {e['csize']/1e9:.2f} GB (method {e['method']}) at offset {e['off']}")
    hdr = curl(url, rng=(e["off"], e["off"] + 29))
    assert struct.unpack("<I", hdr[:4])[0] == 0x04034B50
    nl, el = struct.unpack("<HH", hdr[26:30])
    start = e["off"] + 30 + nl + el
    raw = os.path.join(base, "raw"); os.makedirs(raw, exist_ok=True)
    zp = os.path.join(raw, a.seq + ".zip")
    if e["method"] == 0:
        curl(url, out=zp + ".part", rng=(start, start + e["csize"] - 1))
        os.replace(zp + ".part", zp)
    elif e["method"] == 8:  # deflated outer member: inflate the stream
        curl(url, out=zp + ".def", rng=(start, start + e["csize"] - 1))
        d = zlib.decompressobj(-15)
        with open(zp + ".def", "rb") as fi, open(zp, "wb") as fo:
            for b in iter(lambda: fi.read(1 << 24), b""): fo.write(d.decompress(b))
            fo.write(d.flush())
        os.remove(zp + ".def")
    else:
        raise ValueError(e["method"])
    assert os.path.getsize(zp) == e["usize"] and crc_file(zp) == e["crc"], "CRC/size mismatch of the downloaded zip"
    tmp = asl + ".tmp"; shutil.rmtree(tmp, ignore_errors=True)
    subprocess.run(["unzip", "-q", zp, "-d", tmp], check=True)
    shutil.rmtree(asl, ignore_errors=True); os.rename(tmp, asl)
    # some zips unpack as <asl>/mav0 directly (EuRoC ASL layout), others wrap it in a folder: normalise
    if not os.path.isdir(os.path.join(asl, "mav0")):
        sub = [d for d in os.listdir(asl) if os.path.isdir(os.path.join(asl, d, "mav0"))]
        assert sub, os.listdir(asl)
        for f in os.listdir(os.path.join(asl, sub[0])): shutil.move(os.path.join(asl, sub[0], f), asl)
    here = os.path.dirname(os.path.abspath(__file__))
    subprocess.run([sys.executable, os.path.join(here, "gt_to_tum.py"), "euroc",
                    os.path.join(asl, "mav0", "state_groundtruth_estimate0", "data.csv"), gt], check=True)
    if not a.keep_raw: shutil.rmtree(raw, ignore_errors=True)
    print("ready:", base)


if __name__ == "__main__":
    main()
