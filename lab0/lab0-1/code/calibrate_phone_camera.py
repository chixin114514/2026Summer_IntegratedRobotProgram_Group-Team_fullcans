#!/usr/bin/env python3
"""使用 OpenCV 和棋盘格照片标定手机相机。

本程序适配随项目提供的标定板：9 x 6 个方格、8 x 5 个内角点、
方格边长 30 mm。支持：
  1. 从本地摄像头或手机网络视频流采集照片；
  2. 从照片目录计算相机内参和畸变系数；
  3. 使用已有标定文件对单张照片去畸变。
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def read_image(path: Path) -> Optional[np.ndarray]:
    """兼容 Windows 中文路径的图像读取。"""
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    except (OSError, ValueError):
        return None


def write_image(path: Path, image: np.ndarray) -> bool:
    """兼容 Windows 中文路径的图像写入。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix.lower() or ".jpg"
    ok, encoded = cv2.imencode(suffix, image)
    if not ok:
        return False
    try:
        encoded.tofile(str(path))
        return True
    except OSError:
        return False


def find_corners(gray: np.ndarray, board_size: Tuple[int, int]):
    """优先使用更稳健的 SB 检测器，旧版 OpenCV 自动回退。"""
    if hasattr(cv2, "findChessboardCornersSB"):
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE
        flags |= getattr(cv2, "CALIB_CB_EXHAUSTIVE", 0)
        flags |= getattr(cv2, "CALIB_CB_ACCURACY", 0)
        found, corners = cv2.findChessboardCornersSB(gray, board_size, flags=flags)
        if found:
            return True, corners.astype(np.float32)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if found:
        criteria = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
            50,
            0.001,
        )
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return found, corners


def collect_images(folder: Path) -> List[Path]:
    return sorted(
        p for p in folder.rglob("*")
        if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS
    )


def make_object_points(cols: int, rows: int, square_size_mm: float) -> np.ndarray:
    points = np.zeros((rows * cols, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points[:, :2] *= square_size_mm
    return points


def calculate_view_errors(
    object_points: Sequence[np.ndarray],
    image_points: Sequence[np.ndarray],
    rvecs: Sequence[np.ndarray],
    tvecs: Sequence[np.ndarray],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> List[float]:
    errors: List[float] = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, distortion)
        error = cv2.norm(img, projected, cv2.NORM_L2) / np.sqrt(len(projected))
        errors.append(float(error))
    return errors


def run_calibration(
    object_points: Sequence[np.ndarray],
    image_points: Sequence[np.ndarray],
    image_size: Tuple[int, int],
):
    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-8,
    )
    return cv2.calibrateCamera(
        list(object_points),
        list(image_points),
        image_size,
        None,
        None,
        criteria=criteria,
    )


def save_yaml(
    path: Path,
    image_size: Tuple[int, int],
    board_size: Tuple[int, int],
    square_size_mm: float,
    rms: float,
    mean_error: float,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    new_camera_matrix: np.ndarray,
    roi: Tuple[int, int, int, int],
) -> None:
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not storage.isOpened():
        raise OSError("无法写入 YAML 文件：{}".format(path))
    storage.write("image_width", int(image_size[0]))
    storage.write("image_height", int(image_size[1]))
    storage.write("board_inner_corners", np.array(board_size, dtype=np.int32))
    storage.write("square_size_mm", float(square_size_mm))
    storage.write("rms", float(rms))
    storage.write("mean_per_view_error", float(mean_error))
    storage.write("camera_matrix", camera_matrix)
    storage.write("distortion_coefficients", distortion)
    storage.write("optimal_camera_matrix", new_camera_matrix)
    storage.write("valid_roi", np.array(roi, dtype=np.int32))
    storage.release()


def calibrate(args: argparse.Namespace) -> int:
    image_dir = Path(args.images).expanduser().resolve()
    output_dir = Path(args.output).expanduser().resolve()
    corner_dir = output_dir / "detected_corners"
    undistorted_dir = output_dir / "undistorted_images"
    output_dir.mkdir(parents=True, exist_ok=True)
    corner_dir.mkdir(parents=True, exist_ok=True)
    undistorted_dir.mkdir(parents=True, exist_ok=True)

    board_size = (args.board_cols, args.board_rows)
    template_points = make_object_points(
        args.board_cols, args.board_rows, args.square_size_mm
    )
    image_paths = collect_images(image_dir)
    if not image_paths:
        print("错误：照片目录中没有找到图片：{}".format(image_dir), file=sys.stderr)
        return 2

    print("找到 {} 张候选照片，开始检测 {} x {} 个内角点……".format(
        len(image_paths), args.board_cols, args.board_rows
    ))

    object_points: List[np.ndarray] = []
    image_points: List[np.ndarray] = []
    valid_paths: List[Path] = []
    failed_paths: List[Path] = []
    wrong_size_paths: List[Path] = []
    image_size: Optional[Tuple[int, int]] = None

    for index, path in enumerate(image_paths, start=1):
        image = read_image(path)
        if image is None:
            print("[{}/{}] 无法读取：{}".format(index, len(image_paths), path.name))
            failed_paths.append(path)
            continue

        size = (image.shape[1], image.shape[0])
        if image_size is None:
            image_size = size
        if size != image_size:
            print("[{}/{}] 跳过（分辨率不一致 {}，应为 {}）：{}".format(
                index, len(image_paths), size, image_size, path.name
            ))
            wrong_size_paths.append(path)
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = find_corners(gray, board_size)
        if not found:
            print("[{}/{}] 未检测到完整棋盘格：{}".format(
                index, len(image_paths), path.name
            ))
            failed_paths.append(path)
            continue

        object_points.append(template_points.copy())
        image_points.append(corners)
        valid_paths.append(path)

        annotated = image.copy()
        cv2.drawChessboardCorners(annotated, board_size, corners, True)
        write_image(corner_dir / (path.stem + "_corners.jpg"), annotated)
        print("[{}/{}] 成功：{}".format(index, len(image_paths), path.name))

    if image_size is None or len(valid_paths) < args.min_images:
        print(
            "错误：成功检测 {} 张，至少需要 {} 张。请补拍清晰且完整的标定板照片。".format(
                len(valid_paths), args.min_images
            ),
            file=sys.stderr,
        )
        return 3

    rms, camera_matrix, distortion, rvecs, tvecs = run_calibration(
        object_points, image_points, image_size
    )
    errors = calculate_view_errors(
        object_points, image_points, rvecs, tvecs, camera_matrix, distortion
    )

    rejected_paths: List[Path] = []
    if args.reject_threshold > 0:
        keep = [i for i, error in enumerate(errors) if error <= args.reject_threshold]
        reject = [i for i, error in enumerate(errors) if error > args.reject_threshold]
        if reject and len(keep) >= args.min_images:
            rejected_paths = [valid_paths[i] for i in reject]
            print("剔除 {} 张单图误差超过 {:.3f} 像素的照片并重新标定。".format(
                len(reject), args.reject_threshold
            ))
            object_points = [object_points[i] for i in keep]
            image_points = [image_points[i] for i in keep]
            valid_paths = [valid_paths[i] for i in keep]
            rms, camera_matrix, distortion, rvecs, tvecs = run_calibration(
                object_points, image_points, image_size
            )
            errors = calculate_view_errors(
                object_points, image_points, rvecs, tvecs,
                camera_matrix, distortion
            )

    mean_error = float(np.mean(errors))
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, distortion, image_size, args.alpha, image_size
    )
    roi = tuple(int(v) for v in roi)

    yaml_path = output_dir / "camera_calibration.yaml"
    json_path = output_dir / "camera_calibration.json"
    report_path = output_dir / "calibration_report.txt"
    save_yaml(
        yaml_path, image_size, board_size, args.square_size_mm,
        float(rms), mean_error, camera_matrix, distortion,
        new_camera_matrix, roi,
    )

    result: Dict[str, object] = {
        "model": "OpenCV pinhole camera model",
        "image_width": image_size[0],
        "image_height": image_size[1],
        "board_squares": [args.board_cols + 1, args.board_rows + 1],
        "board_inner_corners": [args.board_cols, args.board_rows],
        "square_size_mm": args.square_size_mm,
        "rms": float(rms),
        "mean_per_view_error": mean_error,
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": distortion.reshape(-1).tolist(),
        "distortion_order": ["k1", "k2", "p1", "p2", "k3"],
        "optimal_camera_matrix": new_camera_matrix.tolist(),
        "valid_roi": list(roi),
        "used_images": [str(p) for p in valid_paths],
        "per_view_errors": {
            str(path): error for path, error in zip(valid_paths, errors)
        },
        "rejected_images": [str(p) for p in rejected_paths],
        "detection_failed_images": [str(p) for p in failed_paths],
        "wrong_resolution_images": [str(p) for p in wrong_size_paths],
    }
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    report_lines = [
        "手机相机 OpenCV 标定报告",
        "=" * 32,
        "图像分辨率：{} x {}".format(*image_size),
        "棋盘格：{} x {} 个方格，{} x {} 个内角点".format(
            args.board_cols + 1, args.board_rows + 1,
            args.board_cols, args.board_rows,
        ),
        "方格边长：{:.3f} mm".format(args.square_size_mm),
        "有效照片：{} 张".format(len(valid_paths)),
        "检测失败：{} 张".format(len(failed_paths)),
        "分辨率不一致：{} 张".format(len(wrong_size_paths)),
        "自动剔除：{} 张".format(len(rejected_paths)),
        "OpenCV RMS：{:.6f} 像素".format(float(rms)),
        "平均单图重投影误差：{:.6f} 像素".format(mean_error),
        "",
        "相机内参矩阵 K：",
        np.array2string(camera_matrix, precision=8, suppress_small=False),
        "",
        "畸变系数 [k1, k2, p1, p2, k3]：",
        np.array2string(distortion.reshape(-1), precision=8, suppress_small=False),
        "",
        "单图重投影误差：",
    ]
    report_lines.extend(
        "{:.6f}  {}".format(error, path.name)
        for path, error in sorted(zip(valid_paths, errors), key=lambda item: item[1])
    )
    report_path.write_text("\n".join(report_lines) + "\n", encoding="utf-8")

    for path in valid_paths[: max(0, args.preview_count)]:
        image = read_image(path)
        if image is None:
            continue
        undistorted = cv2.undistort(
            image, camera_matrix, distortion, None, new_camera_matrix
        )
        if args.crop and roi[2] > 0 and roi[3] > 0:
            x, y, width, height = roi
            undistorted = undistorted[y:y + height, x:x + width]
        write_image(undistorted_dir / (path.stem + "_undistorted.jpg"), undistorted)

    print("\n标定完成")
    print("有效照片：{} 张".format(len(valid_paths)))
    print("OpenCV RMS：{:.6f} 像素".format(float(rms)))
    print("平均单图重投影误差：{:.6f} 像素".format(mean_error))
    if mean_error > 1.0:
        print("警告：平均误差大于 1 像素。请检查模糊、反光、标定板平整度和拍摄姿态覆盖。")
    elif mean_error > 0.5:
        print("提示：结果通常可用，但可以补拍更多清晰且覆盖画面边缘的照片以提高精度。")
    else:
        print("质量判断：平均误差小于或等于 0.5 像素，标定质量通常较好。")
    print("\n相机内参矩阵 K：\n{}".format(camera_matrix))
    print("\n畸变系数 [k1, k2, p1, p2, k3]：\n{}".format(
        distortion.reshape(-1)
    ))
    print("\n结果目录：{}".format(output_dir))
    print("  YAML：{}".format(yaml_path.name))
    print("  JSON：{}".format(json_path.name))
    print("  报告：{}".format(report_path.name))
    return 0


def parse_video_source(value: str):
    return int(value) if value.isdigit() else value


def capture(args: argparse.Namespace) -> int:
    output_dir = Path(args.output).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    board_size = (args.board_cols, args.board_rows)
    cap = cv2.VideoCapture(parse_video_source(args.source))
    if args.width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print("错误：无法打开视频源 {}".format(args.source), file=sys.stderr)
        return 2

    saved = len(collect_images(output_dir))
    print("采集窗口已打开：空格=保存检测成功的照片，F=强制保存，Q/Esc=退出")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("错误：无法读取视频帧。", file=sys.stderr)
                return 3
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = find_corners(gray, board_size)
            display = frame.copy()
            if found:
                cv2.drawChessboardCorners(display, board_size, corners, True)
            status = "DETECTED" if found else "NOT DETECTED"
            color = (0, 200, 0) if found else (0, 0, 255)
            cv2.putText(
                display,
                "{} | saved: {} | SPACE save | Q quit".format(status, saved),
                (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2,
                cv2.LINE_AA,
            )
            if display.shape[1] > args.preview_width:
                scale = args.preview_width / display.shape[1]
                display = cv2.resize(display, None, fx=scale, fy=scale)
            cv2.imshow("Phone camera calibration capture", display)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key == ord(" ") and found:
                saved += 1
                path = output_dir / "calibration_{:03d}.jpg".format(saved)
                write_image(path, frame)
                print("已保存：{}".format(path))
            elif key in (ord("f"), ord("F")):
                saved += 1
                path = output_dir / "calibration_{:03d}.jpg".format(saved)
                write_image(path, frame)
                print("已强制保存：{}".format(path))
    finally:
        cap.release()
        cv2.destroyAllWindows()
    print("采集结束，共保存 {} 张照片。".format(saved))
    return 0


def undistort(args: argparse.Namespace) -> int:
    source = Path(args.image).expanduser().resolve()
    destination = Path(args.output).expanduser().resolve()
    image = read_image(source)
    if image is None:
        print("错误：无法读取图片：{}".format(source), file=sys.stderr)
        return 2

    storage = cv2.FileStorage(str(Path(args.calibration).expanduser()), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        print("错误：无法读取标定文件：{}".format(args.calibration), file=sys.stderr)
        return 3
    camera_matrix = storage.getNode("camera_matrix").mat()
    distortion = storage.getNode("distortion_coefficients").mat()
    stored_width = int(storage.getNode("image_width").real())
    stored_height = int(storage.getNode("image_height").real())
    storage.release()

    size = (image.shape[1], image.shape[0])
    if size != (stored_width, stored_height):
        print(
            "错误：图片分辨率 {} 与标定分辨率 {} 不一致。".format(
                size, (stored_width, stored_height)
            ),
            file=sys.stderr,
        )
        return 4
    new_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, distortion, size, args.alpha, size
    )
    corrected = cv2.undistort(image, camera_matrix, distortion, None, new_matrix)
    if args.crop and roi[2] > 0 and roi[3] > 0:
        x, y, width, height = roi
        corrected = corrected[y:y + height, x:x + width]
    if not write_image(destination, corrected):
        print("错误：无法保存图片：{}".format(destination), file=sys.stderr)
        return 5
    print("去畸变图片已保存：{}".format(destination))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="使用 9 x 6 方格（8 x 5 内角点）、30 mm 棋盘格标定手机相机。"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    capture_parser = subparsers.add_parser("capture", help="从摄像头或手机视频流采集标定照片")
    capture_parser.add_argument("--source", default="0", help="摄像头编号或视频流 URL，默认 0")
    capture_parser.add_argument("--output", default="calibration_images", help="照片保存目录")
    capture_parser.add_argument("--board-cols", type=int, default=8, help="横向内角点数，默认 8")
    capture_parser.add_argument("--board-rows", type=int, default=5, help="纵向内角点数，默认 5")
    capture_parser.add_argument("--width", type=int, default=0, help="请求的视频宽度")
    capture_parser.add_argument("--height", type=int, default=0, help="请求的视频高度")
    capture_parser.add_argument("--preview-width", type=int, default=1280, help="预览窗口最大宽度")
    capture_parser.set_defaults(func=capture)

    calibrate_parser = subparsers.add_parser("calibrate", help="从照片目录计算相机参数")
    calibrate_parser.add_argument("--images", default="calibration_images", help="标定照片目录")
    calibrate_parser.add_argument("--output", default="calibration_output", help="结果输出目录")
    calibrate_parser.add_argument("--board-cols", type=int, default=8, help="横向内角点数，默认 8")
    calibrate_parser.add_argument("--board-rows", type=int, default=5, help="纵向内角点数，默认 5")
    calibrate_parser.add_argument("--square-size-mm", type=float, default=30.0, help="方格边长，默认 30 mm")
    calibrate_parser.add_argument("--min-images", type=int, default=8, help="最少成功照片数，默认 8")
    calibrate_parser.add_argument("--reject-threshold", type=float, default=1.0, help="单图误差剔除阈值；0 表示禁用")
    calibrate_parser.add_argument("--preview-count", type=int, default=3, help="输出去畸变示例数量")
    calibrate_parser.add_argument("--alpha", type=float, default=0.0, help="去畸变视野保留系数 0 到 1")
    calibrate_parser.add_argument("--crop", action="store_true", help="裁掉去畸变后的无效黑边")
    calibrate_parser.set_defaults(func=calibrate)

    undistort_parser = subparsers.add_parser("undistort", help="使用 YAML 参数对单张图片去畸变")
    undistort_parser.add_argument("--calibration", required=True, help="camera_calibration.yaml 路径")
    undistort_parser.add_argument("--image", required=True, help="待处理图片路径")
    undistort_parser.add_argument("--output", default="undistorted.jpg", help="输出图片路径")
    undistort_parser.add_argument("--alpha", type=float, default=0.0, help="视野保留系数 0 到 1")
    undistort_parser.add_argument("--crop", action="store_true", help="裁掉无效黑边")
    undistort_parser.set_defaults(func=undistort)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
