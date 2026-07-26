#!/usr/bin/env python3
"""Offline camera intrinsic calibration from BMP images.

Usage:
    python3 scripts/calibrate_from_images.py /opt/MVS/bin/Temp/Data

Reads BMP images from the given directory, filters by resolution,
detects 11x8 chessboard corners (15mm squares), and outputs
camera_matrix + dist_coeffs to config/camera_calib.yaml.
"""

import argparse
import glob
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

# ---- chessboard parameters (must match calibrate.cpp) ----------------------
CHESSBOARD_COLS = 11
CHESSBOARD_ROWS = 8
SQUARE_SIZE_MM = 15.0
CORNER_REFINE_WIN = 15
MIN_SAMPLES = 5
CALIB_FLAGS = cv2.CALIB_FIX_K3 | cv2.CALIB_ZERO_TANGENT_DIST

# ---- target resolution (current camera) ------------------------------------
TARGET_WIDTH = 2448
TARGET_HEIGHT = 2048

OUTPUT_PATH = Path("config/camera_calib.yaml")


def make_object_points():
    """3D coordinates of chessboard corners in mm."""
    objp = np.zeros((CHESSBOARD_COLS * CHESSBOARD_ROWS, 3), np.float32)
    objp[:, :2] = np.mgrid[
        0:CHESSBOARD_COLS, 0:CHESSBOARD_ROWS
    ].T.reshape(-1, 2) * SQUARE_SIZE_MM
    return objp


def main():
    parser = argparse.ArgumentParser(description="Calibrate camera from BMP images")
    parser.add_argument("image_dir", help="Directory containing BMP images")
    parser.add_argument(
        "-o", "--output", default=str(OUTPUT_PATH), help="Output YAML path"
    )
    parser.add_argument(
        "--width", type=int, default=TARGET_WIDTH, help="Target image width"
    )
    parser.add_argument(
        "--height", type=int, default=TARGET_HEIGHT, help="Target image height"
    )
    args = parser.parse_args()

    image_dir = Path(args.image_dir)
    if not image_dir.is_dir():
        print(f"Error: {image_dir} is not a directory", file=sys.stderr)
        sys.exit(1)

    # Collect BMP files
    bmp_files = sorted(glob.glob(str(image_dir / "*.bmp")))
    if not bmp_files:
        print(f"Error: no BMP files in {image_dir}", file=sys.stderr)
        sys.exit(1)

    print(f"Found {len(bmp_files)} BMP files")

    objp = make_object_points()
    obj_points = []
    img_points = []
    image_size = None
    skipped = 0

    for fpath in bmp_files:
        img = cv2.imread(fpath, cv2.IMREAD_COLOR)
        if img is None:
            print(f"  SKIP (unreadable): {Path(fpath).name}")
            skipped += 1
            continue

        h, w = img.shape[:2]
        if w != args.width or h != args.height:
            print(f"  SKIP (resolution {w}x{h}): {Path(fpath).name}")
            skipped += 1
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (w, h)

        found, corners = cv2.findChessboardCorners(
            gray, (CHESSBOARD_COLS, CHESSBOARD_ROWS), None
        )
        if not found:
            print(f"  SKIP (no corners): {Path(fpath).name}")
            skipped += 1
            continue

        corners = cv2.cornerSubPix(
            gray, corners, (CORNER_REFINE_WIN, CORNER_REFINE_WIN), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        obj_points.append(objp)
        img_points.append(corners)
        print(f"  OK ({len(obj_points):3d}): {Path(fpath).name}")

    print(f"\nDetected corners in {len(obj_points)} images, skipped {skipped}")

    if len(obj_points) < MIN_SAMPLES:
        print(
            f"Error: need at least {MIN_SAMPLES} samples, got {len(obj_points)}",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"Running calibration ({image_size[0]}x{image_size[1]}) ...")
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None, flags=CALIB_FLAGS
    )
    print(f"RMS re-projection error: {rms:.6f} px")

    # Iterative outlier rejection: remove worst image, re-calibrate, repeat
    # until RMS < 1.0 px or < MIN_SAMPLES left
    for iteration in range(10):
        errors = []
        for i in range(len(obj_points)):
            projected, _ = cv2.projectPoints(
                obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
            )
            diff = img_points[i].reshape(-1, 2) - projected.reshape(-1, 2)
            err = np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))
            errors.append(err)
        errors = np.array(errors)
        mean_err = np.mean(errors)
        max_err = np.max(errors)
        print(f"  Pass {iteration+1}: RMS={rms:.4f}, mean={mean_err:.4f}, max={max_err:.4f}, n={len(obj_points)}")

        if rms < 1.0:
            break

        # Remove the worst image
        worst_idx = np.argmax(errors)
        if len(obj_points) <= MIN_SAMPLES:
            print(f"  Cannot remove more images (minimum {MIN_SAMPLES} required)")
            break
        print(f"  Removing image {worst_idx+1} (error {errors[worst_idx]:.4f} px)")
        obj_points.pop(worst_idx)
        img_points.pop(worst_idx)
        rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, img_points, image_size, None, None, flags=CALIB_FLAGS
        )

    # Write YAML
    calib_data = {
        "calibration": {
            "image_width": image_size[0],
            "image_height": image_size[1],
            "chessboard": {
                "cols": CHESSBOARD_COLS,
                "rows": CHESSBOARD_ROWS,
                "square_size_mm": SQUARE_SIZE_MM,
            },
            "camera_matrix": camera_matrix.tolist(),
            "dist_coeffs": dist_coeffs.flatten().tolist(),
            "rms_reprojection_error": float(rms),
            "num_samples": len(obj_points),
        }
    }

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        yaml.dump(calib_data, f, default_flow_style=None, sort_keys=False)
    print(f"\nSaved to {out_path}")


if __name__ == "__main__":
    main()
