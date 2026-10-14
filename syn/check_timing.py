#!/usr/bin/env python3
"""Fail the standalone Cyclone V build on missing or violated timing checks."""
import math
from pathlib import Path
import re
import sys

MODELS = {
    "Slow 1100mV 100C Model", "Slow 1100mV -40C Model",
    "Fast 1100mV 100C Model", "Fast 1100mV -40C Model",
}
REQUIRED = {"Setup", "Hold", "Minimum Pulse Width"}


def check(path):
    text = path.read_text()
    blocks = re.findall(
        r"^Type\s*:\s*(.+)\nSlack\s*:\s*(\S+)\nTNS\s*:\s*(\S+)", text, re.M
    )
    if len(blocks) != len(re.findall(r"^Type\s*:", text, re.M)):
        raise ValueError("incomplete timing record")
    records = {}
    for kind, slack, tns in blocks:
        match = re.fullmatch(
            r"(.+ Model) (Setup|Hold|Recovery|Removal|Minimum Pulse Width) '([^']+)'", kind
        )
        if not match:
            raise ValueError(f"unrecognized timing check: {kind}")
        key = match.groups()
        if key in records:
            raise ValueError(f"duplicate timing check: {kind}")
        values = float(slack), float(tns)
        if not all(math.isfinite(v) for v in values):
            raise ValueError(f"non-finite timing value: {kind}")
        records[key] = values
    # This project has one processor clock. A truncated report or missing
    # corner must not look like a pass merely because it has no negative value.
    expected = {(model, kind, "clk") for model in MODELS for kind in REQUIRED}
    missing = sorted(expected - records.keys())
    if missing:
        raise ValueError("missing timing checks: " + "; ".join(" / ".join(k) for k in missing))
    failed = [(key, values) for key, values in records.items() if any(v < 0 for v in values)]
    if failed:
        for (model, kind, clock), (slack, tns) in failed:
            print(f"Timing FAIL: {model}, {kind}, {clock}: slack {slack:+.3f} ns, TNS {tns:+.3f} ns")
        return 1
    setup = min(values[0] for key, values in records.items() if key[1] == "Setup")
    hold = min(values[0] for key, values in records.items() if key[1] == "Hold")
    print(f"Timing PASS: {len(records)} checks; worst setup {setup:+.3f} ns, hold {hold:+.3f} ns")
    return 0


def main():
    if len(sys.argv) > 2:
        print(f"Usage: {sys.argv[0]} [ap68030.sta.summary]", file=sys.stderr)
        return 2
    path = Path(sys.argv[1]) if len(sys.argv) == 2 else Path(__file__).parent / "output/ap68030.sta.summary"
    try:
        return check(path)
    except (OSError, UnicodeError, ValueError) as error:
        print(f"Timing ERROR: {path}: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
