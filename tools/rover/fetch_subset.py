#!/usr/bin/env python3
"""Fetch the T265 subset of a ROVER sequence from the remote ZIP via HTTP Range requests.

ROVER ships one 11-48 GB ZIP per sequence (https://fdm.hs-esslingen.de/schmidt2025rover/<seq>.zip).
We never download the whole zip: the ZIP64 central directory is read with 2 Range requests,
and only the wanted members are pulled in contiguous byte ranges, inflated (stored / deflate),
CRC32-verified against the central directory, and written straight to their final place.

Usage:
  python3 tools/rover/fetch_subset.py list  <seq>             # show members / sizes / decision
  python3 tools/rover/fetch_subset.py fetch <seq> [--stereo auto|yes|no] [--max-gb 5]
  python3 tools/rover/fetch_subset.py calib                   # calibration.zip -> data/rover/raw_calib/
Run sequentially, in background, e.g.
  for s in garden_small_2024-05-29_1 campus_small_2024-02-19 park_2023-07-31; do
      python3 tools/rover/fetch_subset.py fetch $s; done

Output (data/rover/<seq>/):
  raw/cam_left/<t>.png   original PNG names (unix time with decimals) -- moved by convert_asl.py
  raw/cam_left.txt  raw/imu.txt  raw/groundtruth.txt  raw/central_dir_subset.json
  raw/progress.json      resume state (index of next unfinished member per group)
  raw/decision.json      mono/stereo decision and size accounting
Resume: re-run the same command; finished members are skipped (verified by existence+size+crc
state in progress.json), a partially written member is re-fetched. Each member is written
atomically (tmp + rename). Source: HuggingFace mirror iis-esslingen/ROVER (default, ~4 MB/s; the zip is split in 5 GiB
.part-xx files, treated as one virtual file) or --source fdm (original server, only ~0.4 MB/s
when measured). Only curl is used for HTTP (system trust store; python TLS verification
failed against this host on macOS). Peak disk = final size (no raw zip pieces are kept).
"""
import argparse, json, os, shutil, struct, subprocess, sys, time, zlib

BASE = "https://fdm.hs-esslingen.de/schmidt2025rover"
ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data", "rover"))
CHUNK = 64 << 20
SLACK = 1024


HF = "https://huggingface.co/datasets/iis-esslingen/ROVER"
HF_NAMES = {  # fdm zip name -> HuggingFace name (same bytes, split into 5 GiB .part-xx files)
    "garden_small_2024-05-29_1": "garden_small_day_2024-05-29_1",
    "campus_small_2024-02-19": "campus_small_winter_2024-02-19",
    "park_2023-07-31": "park_summer_2023-07-31",
}


class Source:
    """Virtual file made of one URL (fdm) or several consecutive .part-xx URLs (HuggingFace)."""
    def __init__(self, parts):  # parts: [(url, size)]
        self.parts = parts
        self.size = sum(s for _, s in parts)

    def read(self, start, end):  # inclusive
        out = []; base = 0
        for url, sz in self.parts:
            a, b = max(start, base), min(end, base + sz - 1)
            if a <= b:
                out.append(curl_range(url, a - base, b - base))
            base += sz
        return b"".join(out)


def make_source(seq, which):
    if which == "hf":
        name = HF_NAMES[seq]
        r = subprocess.run(["curl", "-sSfL", "https://huggingface.co/api/datasets/iis-esslingen/ROVER/tree/main?limit=1000"],
                           capture_output=True, text=True, check=True)
        files = sorted((f["path"], f["size"]) for f in json.loads(r.stdout) if f["path"].startswith(name + ".zip.part-"))
        assert files, name
        return Source([(f"{HF}/resolve/main/{p}", sz) for p, sz in files])
    url = f"{BASE}/{seq}.zip"
    return Source([(url, remote_size(url))])


def curl_range(url, start, end, tries=6):
    """Fetch bytes [start,end] inclusive."""
    for k in range(tries):
        cmd = ["curl", "-sS", "-f", "-L", "--connect-timeout", "30", "--max-time", "900",
               "-r", f"{start}-{end}", url]
        r = subprocess.run(cmd, capture_output=True)
        if r.returncode == 0 and len(r.stdout) == end - start + 1:
            return r.stdout
        # server may clamp end past EOF
        if r.returncode == 0 and len(r.stdout) > 0 and len(r.stdout) < end - start + 1:
            return r.stdout
        time.sleep(2 * (k + 1))
    raise RuntimeError(f"range {start}-{end} failed: {r.stderr[-200:]!r}")


def remote_size(url):
    r = subprocess.run(["curl", "-sSfLI", url], capture_output=True, text=True)
    n = [l.split(":")[1].strip() for l in r.stdout.splitlines() if l.lower().startswith("content-length")]
    return int(n[-1])


def read_central_directory(src):
    size = src.size
    tail_len = min(size, 70000)
    tail = src.read(size - tail_len, size - 1)
    i = tail.rfind(b"PK\x05\x06")
    (_, _, _, nent, _, cd_size, cd_off, _) = struct.unpack("<IHHHHIIH", tail[i:i + 22])
    if cd_off == 0xFFFFFFFF or nent == 0xFFFF or cd_size == 0xFFFFFFFF:
        j = tail.rfind(b"PK\x06\x07")
        (_, _, eocd64_off, _) = struct.unpack("<IIQI", tail[j:j + 20])
        rel = eocd64_off - (size - tail_len)
        if 0 <= rel < len(tail):
            e = tail[rel:rel + 56]
        else:
            e = src.read(eocd64_off, eocd64_off + 55)
        (_, _, _, _, _, _, nent, _, cd_size, cd_off) = struct.unpack("<IQHHIIQQQQ", e)
    cd = b""
    pos = cd_off
    while pos < cd_off + cd_size:  # fetch in 16 MB slices
        e = min(cd_off + cd_size, pos + (16 << 20)) - 1
        cd += src.read(pos, e)
        pos = e + 1
    ents = []
    p = 0
    while p < len(cd):
        (sig, _, _, flag, meth, _, _, crc, csz, usz, nl, el, cl, _, _, _, lho) = struct.unpack(
            "<IHHHHHHIIIHHHHHII", cd[p:p + 46])
        assert sig == 0x02014B50
        name = cd[p + 46:p + 46 + nl].decode("utf-8", "replace")
        ex = cd[p + 46 + nl:p + 46 + nl + el]
        q = 0
        while q + 4 <= len(ex):
            hid, hsz = struct.unpack("<HH", ex[q:q + 4])
            if hid == 1:
                d = ex[q + 4:q + 4 + hsz]
                k = 0
                if usz == 0xFFFFFFFF: usz = struct.unpack("<Q", d[k:k + 8])[0]; k += 8
                if csz == 0xFFFFFFFF: csz = struct.unpack("<Q", d[k:k + 8])[0]; k += 8
                if lho == 0xFFFFFFFF: lho = struct.unpack("<Q", d[k:k + 8])[0]; k += 8
            q += 4 + hsz
        ents.append(dict(name=name, method=meth, crc=crc, csize=csz, usize=usz, off=lho))
        p += 46 + nl + el + cl
    assert len(ents) == nent, (len(ents), nent)
    return size, ents


def select(ents):
    """Group wanted members. Returns {group: [entries sorted by offset]}."""
    groups = {"cam_left": [], "cam_right": [], "text": []}
    for e in ents:
        n = e["name"]
        if n.endswith("/"):
            continue
        if "/realsense_T265/cam_left/" in "/" + n and n.endswith(".png"):
            groups["cam_left"].append(e)
        elif "/realsense_T265/cam_right/" in "/" + n and n.endswith(".png"):
            groups["cam_right"].append(e)
        elif n.endswith("realsense_T265/cam_left.txt") or n.endswith("realsense_T265/imu/imu.txt") \
                or n.endswith("groundtruth.txt") or n.endswith("realsense_T265/cam_right.txt"):
            groups["text"].append(e)
    for g in groups.values():
        g.sort(key=lambda e: e["off"])
    return groups


def contiguous(g):
    gaps = 0
    for a, b in zip(g, g[1:]):
        hdr = 30 + len(a["name"].encode())
        if b["off"] - a["off"] - a["csize"] - hdr > 1024:
            gaps += 1
    return gaps


def out_path(seqdir, e):
    n = e["name"]
    if "/realsense_T265/cam_left/" in "/" + n:
        return os.path.join(seqdir, "raw", "cam_left", os.path.basename(n))
    if "/realsense_T265/cam_right/" in "/" + n:
        return os.path.join(seqdir, "raw", "cam_right", os.path.basename(n))
    if n.endswith("imu.txt"): return os.path.join(seqdir, "raw", "imu.txt")
    if n.endswith("cam_left.txt"): return os.path.join(seqdir, "raw", "cam_left.txt")
    if n.endswith("cam_right.txt"): return os.path.join(seqdir, "raw", "cam_right.txt")
    return os.path.join(seqdir, "raw", os.path.basename(n))


def decode_member(buf, base, e):
    """buf[base:] starts at the member's local header."""
    sig, = struct.unpack("<I", buf[base:base + 4])
    assert sig == 0x04034B50, "bad local header"
    nl, el = struct.unpack("<HH", buf[base + 26:base + 30])
    ds = base + 30 + nl + el
    comp = buf[ds:ds + e["csize"]]
    if len(comp) != e["csize"]:
        raise EOFError
    if e["method"] == 0: data = comp
    elif e["method"] == 8: data = zlib.decompress(comp, -15)
    else: raise ValueError(f"method {e['method']}")
    if len(data) != e["usize"] or (zlib.crc32(data) & 0xFFFFFFFF) != e["crc"]:
        raise ValueError(f"CRC/size mismatch {e['name']}")
    return data


def plan_chunks(g, i):
    """Split g[i:] into (first, last_excl, start_byte, end_byte) ranges of ~CHUNK bytes (new range on big gaps)."""
    plan = []
    size_cap = None
    while i < len(g):
        j = i; need = 0
        while j < len(g) and need < CHUNK:
            if j > i and g[j]["off"] - (g[j - 1]["off"] + g[j - 1]["csize"]) > (1 << 20):
                break  # big gap (non-contiguous member): separate request
            need += g[j]["csize"] + 30 + len(g[j]["name"]) + 40; j += 1
        last = g[j - 1]
        plan.append((i, j, g[i]["off"], last["off"] + 30 + len(last["name"].encode()) + SLACK + last["csize"] - 1))
        i = j
    return plan


def fetch_group(src, seqdir, g, key, prog, size, jobs=4):
    """Download+verify+write members of group g in order; `jobs` ranges are prefetched in parallel."""
    from concurrent.futures import ThreadPoolExecutor
    plan = plan_chunks(g, prog.get(key, 0))
    t0 = time.time(); got = 0
    with ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = [ex.submit(src.read, st, min(en, size - 1)) for (_, _, st, en) in plan[:jobs]]
        nxt = jobs
        for n, (i, j, start, _) in enumerate(plan):
            buf = futs[n].result(); futs[n] = None
            if nxt < len(plan):
                futs.append(ex.submit(src.read, plan[nxt][2], min(plan[nxt][3], size - 1))); nxt += 1
            for k in range(i, j):
                e = g[k]
                data = decode_member(buf, e["off"] - start, e)
                p = out_path(seqdir, e)
                os.makedirs(os.path.dirname(p), exist_ok=True)
                with open(p + ".tmp", "wb") as f: f.write(data)
                os.replace(p + ".tmp", p)
                got += e["csize"]
            prog[key] = j
            json.dump(prog, open(os.path.join(seqdir, "raw", "progress.json"), "w"))
            print(f"[{key}] {j}/{len(g)} members, {got/1e6:.0f} MB in {time.time()-t0:.0f}s", flush=True)


def cmd_calib():
    d = os.path.join(ROOT, "raw_calib"); os.makedirs(d, exist_ok=True)
    subprocess.run(["curl", "-sSfL", "-o", os.path.join(d, "calibration.zip"), f"{BASE}/calibration.zip"], check=True)
    subprocess.run(["unzip", "-o", "-q", os.path.join(d, "calibration.zip"), "-d", d], check=True)
    print("calibration ->", d)


def cmd_fetch(seq, stereo, max_gb, list_only=False, source="hf"):
    src = make_source(seq, source)
    seqdir = os.path.join(ROOT, seq); os.makedirs(os.path.join(seqdir, "raw"), exist_ok=True)
    size, ents = read_central_directory(src)
    groups = select(ents)
    sz = {k: sum(e["csize"] for e in v) for k, v in groups.items()}
    mono_total = sz["cam_left"] + sz["text"]
    stereo_total = mono_total + sz["cam_right"]
    use_right = (stereo == "yes") or (stereo == "auto" and stereo_total <= max_gb * 1e9)
    dec = dict(seq=seq, zip_bytes=size, n_left=len(groups["cam_left"]), n_right=len(groups["cam_right"]),
               left_png_bytes=sz["cam_left"], right_png_bytes=sz["cam_right"], text_bytes=sz["text"],
               mono_total=mono_total, stereo_total=stereo_total, max_gb=max_gb,
               gaps_left=contiguous(groups["cam_left"]), mode="stereo" if use_right else "mono",
               reason=("stereo fits" if use_right else "mono only: stereo would be %.2f GB > %.1f GB" %
                       (stereo_total / 1e9, max_gb)))
    print(json.dumps(dec, indent=1), flush=True)
    json.dump(dec, open(os.path.join(seqdir, "raw", "decision.json"), "w"), indent=1)
    json.dump({k: v for k, v in groups.items()}, open(os.path.join(seqdir, "raw", "central_dir_subset.json"), "w"))
    if list_only: return
    need = (stereo_total if use_right else mono_total)
    free = shutil.disk_usage(seqdir).free
    assert free > need * 1.2 + 2e9, f"not enough disk: free {free/1e9:.1f} GB, need {need/1e9:.1f} GB"
    assert mono_total <= max_gb * 1e9, "mono subset exceeds the per-sequence limit"
    pp = os.path.join(seqdir, "raw", "progress.json")
    prog = json.load(open(pp)) if os.path.exists(pp) else {}
    fetch_group(src, seqdir, groups["text"], "text", prog, size)
    fetch_group(src, seqdir, groups["cam_left"], "cam_left", prog, size)
    if use_right: fetch_group(src, seqdir, groups["cam_right"], "cam_right", prog, size)
    open(os.path.join(seqdir, "raw", "FETCH_DONE"), "w").write(time.ctime())
    print("DONE", seq)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["list", "fetch", "calib"])
    ap.add_argument("seq", nargs="?")
    ap.add_argument("--stereo", default="auto", choices=["auto", "yes", "no"])
    ap.add_argument("--source", default="hf", choices=["hf", "fdm"])
    ap.add_argument("--max-gb", type=float, default=5.0)
    a = ap.parse_args()
    if a.cmd == "calib": cmd_calib()
    else: cmd_fetch(a.seq, a.stereo, a.max_gb, list_only=(a.cmd == "list"), source=a.source)
