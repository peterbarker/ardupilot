#!/usr/bin/env python3
import re
import sys

pattern = re.compile(r"^Log (\d+) .* lastLog \1 ")

for line in sys.stdin:
    line = line.rstrip('\n')
    if pattern.search(line):
        print(f"match: {line!r}")
    else:
        print(f"no match: {line!r}")
