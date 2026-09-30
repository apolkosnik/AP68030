#!/usr/bin/env python3
"""Run the WinUAE cputest 68030 corpus (data format v20) against AP030 RTL.

The corpus is WinUAE's (cputest by Toni Wilen): for every instruction and
addressing mode it records input state and the resulting registers, SR,
memory writes and exception frames.  Each slice is expanded by replay_gen.py
(from the AP040 harness) into APR2 records and replayed on the real
ap030_top by tb_cputest.sv under Verilator.

Examples:
  ./run_cputest.py /home/adam/Downloads/data_030.zip                 # every integer group
  ./run_cputest.py data --group Default --instruction 'ADD*'
  ./run_cputest.py data --group IRQ --instruction NOP --slice 0001 --verbose
"""

from __future__ import annotations

import argparse
import concurrent.futures
import fnmatch
import gzip
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import zipfile

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from replay_gen import generate  # noqa: E402

RTL = HERE.parent.parent / "rtl"
SOURCES = ["ap030_top.v", "ap030_core.v", "ap030_memsys.v", "ap030_mmu.v", "ap030_cache.v",
           "ap030_bus.v", "ap030_alu.v", "ap030_muldiv.v", "ap030_regfile.v"]
# FPU groups need a 68881/68882 coprocessor, which the AP030 does not have
# v20 zips name groups 68030_<name>; merged (.daz) sets name them 3_<name>
INTEGER_GROUPS = ["AE", "Basic", "Default", "FFEXT_DST", "FFEXT_SRC", "IRQ", "JITLM",
                  "ODD_EXC", "ODD_IRQ", "ODD_STK",
                  "BASIC", "EXTDST", "EXTSRC", "ODDEXC", "ODDIRQ", "ODDSTK"]
GROUP_RE = re.compile(r"^(?:68030|3)_(.+)$")
MRG = b"\xafMRG"


def merged_chunks(path: Path):
    """Split a merged .daz file (WinUAE cputest main.c load_file_offset):
    a sequence of {"\xafMRG", big-endian size, gzip image}; anything else ends
    it.  Chunk 0 is the instruction header (0000.dat), then the slices."""
    d = path.read_bytes()
    o, out = 0, []
    while o + 8 <= len(d) and d[o:o + 4] == MRG:
        n = int.from_bytes(d[o + 4:o + 8], "big")
        if n == 0:
            break
        out.append(gzip.decompress(d[o + 8:o + 8 + n]))
        o += 8 + n
    return out


def corpus_root(source: Path, work: Path) -> Path:
    if source.is_file():
        dest = work / "corpus"
        stamp = dest / ".extracted"
        if not stamp.exists():
            if dest.exists():
                shutil.rmtree(dest)
            dest.mkdir(parents=True)
            with zipfile.ZipFile(source) as zf:
                zf.extractall(dest)
            stamp.write_text(str(source))
        source = dest
    for sub in ("data", "data030"):
        if (source / sub).is_dir():
            source = source / sub
    if not any(GROUP_RE.match(p.name) for p in source.iterdir() if p.is_dir()):
        raise SystemExit("no 68030_*/3_* groups below %s" % source)
    return source


def gunzip_to(src: Path, dst: Path) -> Path:
    if not dst.exists():
        dst.write_bytes(gzip.decompress(src.read_bytes()))
    return dst


def build(work: Path, rebuild: bool) -> Path:
    exe = work / "obj" / "tb_cputest"
    if exe.exists() and not rebuild:
        return exe
    cmd = ["verilator", "--binary", "--timing", "-Wno-fatal", "-Wno-lint", "-Wno-style", "-Wno-WIDTH",
           "-Wno-TIMESCALEMOD", "-Wno-CASEINCOMPLETE", "-Wno-MULTIDRIVEN", "-O2",
           "-I%s" % RTL, "-I%s" % (RTL / "core"), "--top-module", "tb_cputest",
           "--Mdir", str(work / "obj"), "-o", "tb_cputest", str(HERE / "tb_cputest.sv")] + \
          [str(RTL / s) for s in SOURCES]
    log = work / "build.log"
    with log.open("w") as f:
        r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        raise SystemExit("verilator build failed, see %s" % log)
    return exe


def discover(root: Path, args):
    items = []
    for gdir in sorted(p for p in root.iterdir() if p.is_dir()):
        m = GROUP_RE.match(gdir.name)
        if not m:
            continue
        group = m.group(1)
        if group not in INTEGER_GROUPS or not any(fnmatch.fnmatchcase(group, p) for p in args.group):
            continue
        for idir in sorted(p for p in gdir.iterdir() if p.is_dir()):
            if not any(fnmatch.fnmatchcase(idir.name, p) for p in args.instruction):
                continue
            daz = idir / "0000.daz"
            if daz.exists():
                n = len(merged_chunks(daz)) - 1
                for k in range(1, n + 1):
                    num = "%04d" % k
                    if any(fnmatch.fnmatchcase(num, p) for p in args.slice):
                        items.append(dict(group=group, gdir=gdir, instruction=idir.name, idir=idir,
                                          slice=num, path=daz, chunk=k))
                continue
            for sl in sorted(idir.glob("[0-9][0-9][0-9][0-9].dat*")):
                num = sl.name[:4]
                if num == "0000" or not any(fnmatch.fnmatchcase(num, p) for p in args.slice):
                    continue
                items.append(dict(group=group, gdir=gdir, instruction=idir.name, idir=idir,
                                  slice=num, path=sl))
    return items


def group_mem(gdir: Path, mem: Path, name: str) -> Path:
    gz = gdir / (name + ".gz")
    if gz.exists():
        return gunzip_to(gz, mem / name)
    return gdir / name


def run_slice(item, exe: Path, work: Path, limit):
    tag = "%s_%s_%s" % (item["group"], re.sub(r"[^A-Za-z0-9.]", "_", item["instruction"]), item["slice"])
    jobdir = work / "jobs"
    mem = work / "mem" / item["group"]
    mem.mkdir(parents=True, exist_ok=True)
    lmem = group_mem(item["gdir"], mem, "lmem.dat")
    tmem = group_mem(item["gdir"], mem, "tmem.dat")
    dat = item["path"]
    header = item["idir"] / "0000.dat"
    if "chunk" in item:
        chunks = merged_chunks(dat)
        header = jobdir / (tag + ".hdr")
        header.write_bytes(chunks[0])
        plain = jobdir / (tag + ".dat")
        plain.write_bytes(chunks[item["chunk"]])
        dat = plain
    elif dat.suffix == ".gz":
        plain = jobdir / (tag + ".dat")
        plain.write_bytes(gzip.decompress(dat.read_bytes()))
        dat = plain
    job = jobdir / (tag + ".apr")
    try:
        info = generate(str(header), str(dat), str(job), max_rounds=limit)
    except Exception as e:  # a corpus record the generator does not understand
        return dict(item, status="error", message="replay_gen: %s" % e, rounds=0, mism=0)
    r = subprocess.run([str(exe), "+job=%s" % job, "+lmem=%s" % lmem, "+tmem=%s" % tmem],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=3600)
    out = r.stdout
    (work / "logs" / (tag + ".log")).write_text(out)
    m = re.search(r"dat replay: (\d+) rounds, (\d+) mismatches, (\d+) harness errors", out)
    job.unlink(missing_ok=True)
    if dat != item["path"]:
        dat.unlink(missing_ok=True)
    if header.parent == jobdir:
        header.unlink(missing_ok=True)
    if not m:
        return dict(item, status="error", message=out[-400:], rounds=0, mism=0)
    rounds, mism, herr = map(int, m.groups())
    sk = re.search(r"irq-order rounds not compared: (\d+)", out)
    status = "pass" if mism == 0 and herr == 0 else "fail"
    first = "\n".join(l for l in out.splitlines() if l.startswith(("MISMATCH", "FAIL")))[:1200]
    return dict(item, status=status, rounds=rounds, mism=mism + herr, message=first,
                skips=int(sk.group(1)) if sk else 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("corpus", type=Path)
    ap.add_argument("--work", type=Path, default=HERE / "work")
    ap.add_argument("--group", action="append", default=[])
    ap.add_argument("--instruction", action="append", default=[])
    ap.add_argument("--slice", action="append", default=[])
    ap.add_argument("--limit", type=int, help="records per slice")
    ap.add_argument("--jobs", type=int, default=min(os.cpu_count() or 1, 24))
    ap.add_argument("--rebuild", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    args.group = args.group or ["*"]
    args.instruction = args.instruction or ["*"]
    args.slice = args.slice or ["*"]
    work = args.work.resolve()
    for d in ("jobs", "logs", "mem"):
        (work / d).mkdir(parents=True, exist_ok=True)
    root = corpus_root(args.corpus.resolve(), work)
    exe = build(work, args.rebuild)
    items = discover(root, args)
    print("slices:", len(items), flush=True)
    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        futs = [ex.submit(run_slice, it, exe, work, args.limit) for it in items]
        for f in concurrent.futures.as_completed(futs):
            r = f.result()
            results.append(r)
            if r["status"] != "pass" and args.verbose:
                print("%-5s %s/%s/%s %s" % (r["status"], r["group"], r["instruction"], r["slice"], r["message"]), flush=True)
    groups = {}
    for r in results:
        g = groups.setdefault(r["group"], dict(slices=0, fail=0, rounds=0, mism=0, skips=0, bad=set()))
        g["slices"] += 1
        g["skips"] += r.get("skips", 0)
        g["rounds"] += r.get("rounds", 0)
        g["mism"] += r.get("mism", 0)
        if r["status"] != "pass":
            g["fail"] += 1
            g["bad"].add(r["instruction"])
    total_fail = 0
    for name in sorted(groups):
        g = groups[name]
        total_fail += g["fail"]
        print("%-10s slices %5d  failing %4d  rounds %8d  mismatches %7d  irq-order %5d  %s" %
              (name, g["slices"], g["fail"], g["rounds"], g["mism"], g["skips"], " ".join(sorted(g["bad"]))[:300]))
    with (work / "results.txt").open("w") as f:
        for r in sorted(results, key=lambda r: (r["group"], r["instruction"], r["slice"])):
            f.write("%s %s/%s/%s rounds=%d mism=%d\n%s\n" % (r["status"], r["group"], r["instruction"],
                                                             r["slice"], r.get("rounds", 0), r.get("mism", 0),
                                                             r.get("message", "")))
    print("details: %s" % (work / "results.txt"))
    sys.exit(1 if total_fail else 0)


if __name__ == "__main__":
    main()
