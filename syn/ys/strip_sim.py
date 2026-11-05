import re, sys
# drop "// synthesis translate_off" .. "// synthesis translate_on" regions
# (nesting counted), leaving the line count alone
depth = 0
out = []
for line in open(sys.argv[1]).read().split("\n"):
    m = re.match(r"\s*//\s*(synthesis|synopsys|pragma)\s+translate_(off|on)", line)
    if m:
        depth += 1 if m.group(2) == "off" else -1
        depth = max(depth, 0)
        out.append("")
        continue
    out.append("" if depth else line)
print("\n".join(out))
