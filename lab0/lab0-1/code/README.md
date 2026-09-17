# 手机相机 OpenCV 内参标定

本程序适配随项目提供的 A4 棋盘格：

- 9 x 6 个方格
- 8 x 5 个内角点（OpenCV 参数）
- 方格边长 30 mm

## 1. 打印和拍照

1. 用 A4 横向、100% 实际尺寸打印 PDF，关闭“适合页面”或自动缩放。
2. 用尺子确认底部 100 mm 检查线确实为 100 mm。
3. 将纸张贴在平整硬板上，不能弯曲。
4. 使用同一个手机镜头、相同分辨率、相同变焦倍率拍摄 20～30 张。
5. 让棋盘格出现在画面中央、四角和边缘，并改变距离和 15°～45° 倾角。
6. 每张照片必须包含完整棋盘格，避免模糊、反光和过曝。
7. 将原始照片复制到 `calibration_images/`，不要通过社交软件压缩。

## 2. 安装依赖

在本目录打开终端：

```bash
python3 -m pip install -r requirements.txt
```

## 3. 使用已经拍好的照片标定

将照片放入 `calibration_images/` 后，可以直接运行：

```bash
bash run_calibration.sh
```

也可以手动指定目录：

```bash
python3 calibrate_phone_camera.py calibrate \
  --images calibration_images \
  --output calibration_output
```

程序默认使用 8 x 5 个内角点和 30 mm 方格边长，不需要额外填写。

输出目录包含：

```text
calibration_output/
├── camera_calibration.yaml
├── camera_calibration.json
├── calibration_report.txt
├── detected_corners/
└── undistorted_images/
```

重点查看：

- `camera_calibration.yaml`：可供 OpenCV 或机器人程序直接读取。
- `camera_calibration.json`：便于人工阅读和其他语言解析。
- `calibration_report.txt`：内参矩阵、畸变系数、总误差和每张照片误差。
- `detected_corners/`：确认角点检测是否正确。
- `undistorted_images/`：查看去畸变效果。

## 4. 从手机网络视频流直接采集（可选）

若手机应用提供 MJPEG/HTTP 视频地址，例如 `http://192.168.1.20:8080/video`：

```bash
python3 calibrate_phone_camera.py capture \
  --source http://192.168.1.20:8080/video \
  --output calibration_images
```

若手机通过 USB 被系统识别为普通摄像头：

```bash
python3 calibrate_phone_camera.py capture --source 0
```

采集窗口按键：

- 空格：仅在成功检测完整棋盘格时保存。
- `F`：强制保存当前画面。
- `Q` 或 `Esc`：结束采集。

## 5. 对单张照片去畸变

```bash
python3 calibrate_phone_camera.py undistort \
  --calibration calibration_output/camera_calibration.yaml \
  --image test.jpg \
  --output test_undistorted.jpg
```

## 6. 结果判断

- 平均单图重投影误差小于 0.5 像素：通常很好。
- 0.5～1.0 像素：通常可用。
- 大于 1.0 像素：检查照片是否模糊、标定板是否弯曲、角点是否覆盖图像边缘。

标定参数只适用于标定时的同一镜头、图像分辨率、变焦倍率和裁剪模式。手机的主摄、超广角和长焦镜头必须分别标定。
