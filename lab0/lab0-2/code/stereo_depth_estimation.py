#!/usr/bin/env python3
"""Stereo depth estimation with OpenCV's aloe stereo sample pair.

The sample pair is already rectified, so corresponding pixels are searched
along horizontal epipolar lines.  StereoSGBM computes disparity and the
standard relation Z = f B / d converts disparity to depth under a supplied
baseline B and focal length f.

The OpenCV aloe pair does not ship with a physical camera calibration.  The
default baseline is therefore an explicit demonstration value (0.10 m) and
the resulting depth scale is only meaningful after replacing it with the
actual stereo baseline and focal length.
"""

from __future__ import annotations

import argparse
import json
import platform
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
DEFAULT_LEFT = ROOT / "data" / "aloeL.jpg"
DEFAULT_RIGHT = ROOT / "data" / "aloeR.jpg"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Estimate disparity, depth, and a point cloud from a rectified stereo pair."
    )
    parser.add_argument("--left", type=Path, default=DEFAULT_LEFT, help="left image path")
    parser.add_argument("--right", type=Path, default=DEFAULT_RIGHT, help="right image path")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "output", help="output directory")
    parser.add_argument("--scale", type=float, default=0.5, help="image scale, e.g. 0.5 for faster processing")
    parser.add_argument("--min-disparity", type=int, default=0)
    parser.add_argument("--num-disparities", type=int, default=128, help="multiple of 16")
    parser.add_argument("--block-size", type=int, default=5, help="odd SGBM matching window size")
    parser.add_argument("--baseline", type=float, default=0.10, help="stereo baseline in metres (demo default)")
    parser.add_argument(
        "--focal-px",
        type=float,
        default=None,
        help="focal length in pixels; default is 0.8 times the resized image width",
    )
    return parser.parse_args()


def read_pair(left_path: Path, right_path: Path, scale: float) -> tuple[np.ndarray, np.ndarray]:
    left = cv2.imread(str(left_path), cv2.IMREAD_COLOR)
    right = cv2.imread(str(right_path), cv2.IMREAD_COLOR)
    if left is None:
        raise FileNotFoundError(f"cannot read left image: {left_path}")
    if right is None:
        raise FileNotFoundError(f"cannot read right image: {right_path}")
    if left.shape[:2] != right.shape[:2]:
        raise ValueError(f"left/right image sizes differ: {left.shape[:2]} vs {right.shape[:2]}")
    if not (0 < scale <= 1.0):
        raise ValueError("--scale must be in the interval (0, 1]")
    if scale != 1.0:
        size = (round(left.shape[1] * scale), round(left.shape[0] * scale))
        left = cv2.resize(left, size, interpolation=cv2.INTER_AREA)
        right = cv2.resize(right, size, interpolation=cv2.INTER_AREA)
    return left, right


def build_matcher(args: argparse.Namespace, channels: int) -> cv2.StereoSGBM:
    if args.num_disparities <= 0 or args.num_disparities % 16:
        raise ValueError("--num-disparities must be a positive multiple of 16")
    if args.block_size < 3 or args.block_size % 2 == 0:
        raise ValueError("--block-size must be an odd integer >= 3")
    window = args.block_size
    return cv2.StereoSGBM_create(
        minDisparity=args.min_disparity,
        numDisparities=args.num_disparities,
        blockSize=window,
        P1=8 * channels * window * window,
        P2=32 * channels * window * window,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )


def save_point_cloud(path: Path, points: np.ndarray, colors: np.ndarray, valid: np.ndarray) -> int:
    xyz = points[valid]
    rgb = cv2.cvtColor(colors, cv2.COLOR_BGR2RGB)[valid]
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    rgb = rgb[finite]
    # Remove extreme values so that a few invalid boundary estimates do not
    # dominate the point-cloud viewer's auto-scaling.
    if xyz.size:
        keep = np.abs(xyz[:, 2]) < np.percentile(np.abs(xyz[:, 2]), 99.5)
        xyz, rgb = xyz[keep], rgb[keep]
    header = (
        "ply\nformat ascii 1.0\n"
        f"element vertex {len(xyz)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
    )
    with path.open("w", encoding="ascii") as stream:
        stream.write(header)
        for point, color in zip(xyz, rgb):
            stream.write(
                f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
            )
    return len(xyz)


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    left, right = read_pair(args.left, args.right, args.scale)
    gray_left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

    matcher = build_matcher(args, channels=1)
    disparity = matcher.compute(gray_left, gray_right).astype(np.float32) / 16.0
    h, w = gray_left.shape
    focal_px = args.focal_px if args.focal_px is not None else 0.8 * w
    if args.baseline <= 0:
        raise ValueError("--baseline must be positive")

    # A conservative validity mask excludes the SGBM invalid code and points
    # outside the requested disparity interval.
    valid = np.isfinite(disparity) & (disparity > max(0.0, args.min_disparity))
    depth = np.full_like(disparity, np.nan, dtype=np.float32)
    depth[valid] = focal_px * args.baseline / disparity[valid]

    # Disparity visualization: invalid pixels are black; valid values are
    # normalized using the configured search range.
    disp_vis = np.zeros_like(disparity, dtype=np.uint8)
    if np.any(valid):
        lo = max(0.0, float(args.min_disparity))
        hi = lo + float(args.num_disparities)
        scaled = np.clip((disparity - lo) / max(hi - lo, 1.0), 0.0, 1.0)
        disp_vis[valid] = np.uint8(255.0 * scaled[valid])
    disparity_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_TURBO)
    disparity_color[~valid] = (0, 0, 0)

    # Clip only for visualization; the raw depth array is also saved in NPZ.
    depth_vis = np.zeros_like(disparity, dtype=np.uint8)
    if np.any(valid):
        low, high = np.percentile(depth[valid], [2, 98])
        normalized = np.clip((depth - low) / max(high - low, 1e-9), 0.0, 1.0)
        depth_vis[valid] = np.uint8(255.0 * (1.0 - normalized[valid]))
    depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_TURBO)
    depth_color[~valid] = (0, 0, 0)

    # The Q matrix is the rectified stereo model with the supplied focal and
    # baseline.  reprojectImageTo3D returns coordinates in metres when B is m.
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    Q = np.float32(
        [
            [1, 0, 0, -cx],
            [0, 1, 0, -cy],
            [0, 0, 0, focal_px],
            [0, 0, 1.0 / args.baseline, 0],
        ]
    )
    points_3d = cv2.reprojectImageTo3D(disparity, Q)

    cv2.imwrite(str(output_dir / "left.png"), left)
    cv2.imwrite(str(output_dir / "right.png"), right)
    cv2.imwrite(str(output_dir / "disparity_color.png"), disparity_color)
    cv2.imwrite(str(output_dir / "depth_color.png"), depth_color)
    np.savez_compressed(output_dir / "depth_arrays.npz", disparity=disparity, depth=depth, valid=valid)
    point_count = save_point_cloud(output_dir / "point_cloud.ply", points_3d, left, valid)

    summary = {
        "left_image": str(args.left),
        "right_image": str(args.right),
        "input_size": [int(left.shape[1]), int(left.shape[0])],
        "scale": args.scale,
        "algorithm": "OpenCV StereoSGBM (SGBM_3WAY)",
        "python_version": platform.python_version(),
        "opencv_version": cv2.__version__,
        "numpy_version": np.__version__,
        "min_disparity": args.min_disparity,
        "num_disparities": args.num_disparities,
        "block_size": args.block_size,
        "focal_length_px": float(focal_px),
        "baseline_m": float(args.baseline),
        "valid_pixel_ratio": float(np.count_nonzero(valid) / valid.size),
        "disparity_min_valid_px": float(np.min(disparity[valid])) if np.any(valid) else None,
        "disparity_max_valid_px": float(np.max(disparity[valid])) if np.any(valid) else None,
        "depth_min_m": float(np.nanmin(depth)) if np.any(valid) else None,
        "depth_max_m": float(np.nanmax(depth)) if np.any(valid) else None,
        "point_count": point_count,
        "scale_warning": "The aloe pair has no physical calibration; replace baseline and focal length for metric depth.",
    }
    (output_dir / "run_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Outputs written to: {output_dir}")


if __name__ == "__main__":
    main()
