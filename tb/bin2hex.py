#!/usr/bin/env python3
"""Convert a flat binary into one byte per line of hex for $readmemh."""
import sys

def main():
    if len(sys.argv) != 3:
        print("usage: bin2hex.py <in.bin> <out.hex>")
        return 1
    data = open(sys.argv[1], "rb").read()
    with open(sys.argv[2], "w") as out:
        for b in data:
            out.write("%02x\n" % b)
    return 0

if __name__ == "__main__":
    sys.exit(main())
