#!/usr/bin/env python3
"""Pull DBA-Fusion results that the Colab notebook uploaded as ClearML artifacts.

For every task named "<prefix><dataset>/<seq>" (default prefix "dbafusion/") in the ClearML project (default "vio-bench") the NEWEST task
that has a `traj_tum` artifact (and no "smoke" tag) is taken; its artifacts are copied to
  results/dbafusion/<dataset>/<seq>/{traj_tum.txt, log.txt, run_stats.json, metrics.json, plots/...}
Read-only against ClearML. Needs ~/clearml.conf (or CLEARML_* env vars).

usage: python scripts/fetch_clearml_results.py [--project vio-bench] [--prefix dbafusion/] [--out results/dbafusion]
                                               [--seqs garden,park] [--list-file FILE]
--list-file gets one "<dataset>/<seq>" line per fetched sequence (used by scripts/fetch_colab_results.sh).
"""
import argparse, os, shutil, sys

# artifact name -> file name inside results/dbafusion/<ds>/<seq>/
FILES = {"traj_tum": "traj_tum.txt", "log": "log.txt", "run_stats": "run_stats.json", "metrics": "metrics.json"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--project", default="vio-bench")
    ap.add_argument("--prefix", default="dbafusion/")
    ap.add_argument("--out", default="results/dbafusion")
    ap.add_argument("--seqs", default="", help="comma list, prefix match on the sequence name (default: all)")
    ap.add_argument("--list-file", default=None)
    a = ap.parse_args()
    from clearml import Task
    tasks = Task.get_tasks(project_name=a.project, task_name="^" + a.prefix.replace("/", "\\/"),
                           task_filter={"order_by": ["-last_update"], "system_tags": ["-archived"]})
    want = [s for s in a.seqs.split(",") if s]
    seen, fetched = set(), []
    for t in tasks:
        name = t.name
        if not name.startswith(a.prefix) or name in seen:
            continue
        parts = name[len(a.prefix):].split("/")
        if len(parts) != 2:
            continue
        ds, seq = parts
        if want and not any(seq.startswith(w) for w in want):
            continue
        if "smoke" in (t.get_tags() or []) or "traj_tum" not in t.artifacts:
            continue                       # newest *usable* task wins; smoke/failed runs are skipped
        seen.add(name)
        dst = os.path.join(a.out, ds, seq); os.makedirs(dst, exist_ok=True)
        for art_name, fname in FILES.items():
            art = t.artifacts.get(art_name)
            if art is None:
                continue
            src = art.get_local_copy(force_download=True)
            if src is None:
                print(f"!! {name}: artifact {art_name} could not be downloaded", file=sys.stderr); continue
            shutil.copyfile(src, os.path.join(dst, fname))
        art = t.artifacts.get("plots")     # folder artifact -> ClearML zips it; get_local_copy extracts it
        if art is not None:
            src = art.get_local_copy(force_download=True)
            if src and os.path.isdir(src):
                shutil.rmtree(os.path.join(dst, "plots"), ignore_errors=True)
                shutil.copytree(src, os.path.join(dst, "plots"))
        n = sum(1 for l in open(os.path.join(dst, "traj_tum.txt")) if l.strip())
        print(f"{ds}/{seq}: task {t.id} ({t.get_status()}), {n} poses -> {dst}")
        fetched.append(f"{ds}/{seq}")
    if not fetched:
        print(f"!! no task '{a.prefix}*' with a traj_tum artifact in project '{a.project}' (did the Colab run finish?)", file=sys.stderr)
        sys.exit(1)
    if a.list_file:
        open(a.list_file, "w").write("\n".join(fetched) + "\n")


if __name__ == "__main__":
    main()
