#!/bin/python
#cat src/delp2.txt |./transpose.py
import sys

for c in zip(*(l.split() for l in sys.stdin.readlines() if l.strip())):
	    print(' '.join(c))
