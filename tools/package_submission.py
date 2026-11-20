#!/usr/bin/env python3
"""Clean submission packager (course deliverable ZIP + separate paper PDF).

Builds the ZIP from the FROZEN git commit (explicit file list, no caches,
no local-only results) plus the explicitly listed generated artifacts of the
current batch (rebuilt models, report PDF/figures, metrics export).
Usage:  python tools/package_submission.py --batch full_<id> --out deliver/
"""
import argparse, hashlib, json, os, subprocess, zipfile

SOURCES = [  # controlled source files from the frozen commit
    "main.m", "README.md", "README.txt",
    "matlab/", "simulink/closed_loop.slx", "simulink/plant_test.slx",
]
ARTIFACTS = [  # generated deliverables (batch-dependent)
    "report/main.pdf", "report/main.tex", "report/metrics_macros.tex",
    "report/metrics_table.tex", "report/rho_table.tex", "report/gains_table.tex",
    "report/figs/",
]
RESULTS_KEEP = ["metrics.json"]  # structured metrics ride along; raw runs stay local

EXCLUDE = ("slprj", ".auto", "legacy", "scratch", ".git", "tectonic", "_render")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def collect(root):
    files = []
    for entry in SOURCES + ARTIFACTS:
        full = os.path.join(root, entry)
        if os.path.isdir(full):
            for dp, dns, fns in os.walk(full):
                if any(x in dp for x in EXCLUDE):
                    continue
                files += [os.path.relpath(os.path.join(dp, fn), root) for fn in fns]
        elif os.path.isfile(full):
            files.append(entry)
        else:
            raise SystemExit(f"MISSING deliverable: {entry} (run main('full') first)")
    return sorted(set(files))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--batch", required=True, help="execution_id under results/")
    ap.add_argument("--out", default="deliver")
    a = ap.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sha = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root,
                         capture_output=True, text=True).stdout.strip()
    files = collect(root)
    # batch metrics
    mj = os.path.join(root, "results", a.batch, "metrics.json")
    if os.path.isfile(mj):
        files.append(os.path.relpath(mj, root))
    else:
        raise SystemExit(f"MISSING results/{a.batch}/metrics.json")
    manifest = {"git_head": sha, "batch": a.batch,
                "files": {f: {"sha256": sha256(os.path.join(root, f)),
                              "bytes": os.path.getsize(os.path.join(root, f))}
                          for f in files}}
    os.makedirs(a.out, exist_ok=True)
    zpath = os.path.join(a.out, "AerialRobotics_2026_submission.zip")
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        for f in files:
            z.write(os.path.join(root, f), f)
        z.writestr("MANIFEST.json", json.dumps(manifest, indent=1))
    mpath = os.path.join(a.out, "report_main.pdf")
    if os.path.isfile(os.path.join(root, "report", "main.pdf")):
        with open(os.path.join(root, "report", "main.pdf"), "rb") as s, \
             open(mpath, "wb") as d:
            d.write(s.read())
    mfile = os.path.join(a.out, "MANIFEST.json")
    with open(mfile, "w") as f:
        json.dump({**manifest, "zip": {"path": zpath, "sha256": sha256(zpath),
                                       "bytes": os.path.getsize(zpath)}}, f, indent=1)
    print(f"packaged {len(files)} files -> {zpath}")


if __name__ == "__main__":
    main()
