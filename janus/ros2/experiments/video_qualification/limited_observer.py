#!/usr/bin/env python3
"""Set one OpenCV worker for this shared guest, then run the pinned observer."""
import runpy
import sys

import cv2

cv2.setNumThreads(1)
print(f"OpenCV {cv2.__version__}, worker threads {cv2.getNumThreads()}", flush=True)
script = sys.argv.pop(1)
runpy.run_path(script, run_name="__main__")
