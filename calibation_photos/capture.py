#!/usr/bin/env python3
"""Display live video from a V4L2 USB camera; press SPACE to save a frame.

Example:
    python capture.py /dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0 ./captures
"""

import argparse
import sys
import time
from pathlib import Path
import cv2

FOURCC = "YUYV"
FRAME_WIDTH = 1600
FRAME_HEIGHT = 1304
FPS = 30
IMAGE_EXT = ".png"

WINDOW_NAME = "Camera (SPACE = capture, Q/ESC = quit)"
KEY_SPACE = 32
KEY_ESC = 27


def parse_args():
    p = argparse.ArgumentParser(description="USB camera viewer with snapshot on SPACE.")
    p.add_argument("device", help="Camera device path, e.g. /dev/v4l/by-path/...-video-index0")
    p.add_argument("output_dir", type=Path, help="Directory to save images into")
    return p.parse_args()


def open_camera(device: str) -> cv2.VideoCapture:
    # by-path entries are symlinks; resolve to the real /dev/videoN node.
    real_path = str(Path(device).resolve())
    cap = cv2.VideoCapture(real_path, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera: {device} (resolved to {real_path})")

    # Set FOURCC first, then resolution, so the driver picks the matching mode.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*FOURCC))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if FPS:
        cap.set(cv2.CAP_PROP_FPS, FPS)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (actual_w, actual_h) != (FRAME_WIDTH, FRAME_HEIGHT):
        print(f"Warning: requested {FRAME_WIDTH}x{FRAME_HEIGHT}, camera gave {actual_w}x{actual_h}",
              file=sys.stderr)
    return cap


def next_filename(output_dir: Path) -> Path:
    """Timestamped name, with a counter suffix if it already exists (no overwrite)."""
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = output_dir / f"img_{stamp}{IMAGE_EXT}"
    n = 1
    while path.exists():
        path = output_dir / f"img_{stamp}_{n:02d}{IMAGE_EXT}"
        n += 1
    return path


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    cap = open_camera(args.device)
    saved = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame from camera.", file=sys.stderr)
                break

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF

            if key == KEY_SPACE:
                path = next_filename(args.output_dir)
                if cv2.imwrite(str(path), frame):
                    saved += 1
                    print(f"Saved {path}")
                else:
                    print(f"Failed to write {path}", file=sys.stderr)
            elif key in (KEY_ESC, ord("q")):
                break

            # Exit if the window was closed with the X button.
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
    print(f"Done. {saved} image(s) saved to {args.output_dir}")


if __name__ == "__main__":
    main()