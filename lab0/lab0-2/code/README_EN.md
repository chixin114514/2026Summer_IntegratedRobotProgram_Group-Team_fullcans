# Stereo Depth Estimation with OpenCV Sample Data

This package uses the official OpenCV `aloeL.jpg` and `aloeR.jpg` stereo sample pair. The images are already rectified, so the program searches correspondences along horizontal image rows.

Run the experiment:

```bash
cd stereo_depth_opencv
python3 stereo_depth_estimation.py
```

The default run uses StereoSGBM with `scale=0.5`, `numDisparities=128`, `blockSize=5`, a demonstration focal length of `512.8 px`, and a demonstration baseline of `0.10 m`. Replace the last two values with real stereo-calibration values for metric depth:

```bash
python3 stereo_depth_estimation.py \
  --focal-px 700 \
  --baseline 0.060 \
  --scale 1.0
```

Generated results are written to `output/`: `disparity_color.png`, `depth_color.png`, `point_cloud.ply`, `depth_arrays.npz`, and `run_summary.json`. The tested run produced a 70.79% valid-disparity ratio and 249,334 finite point-cloud vertices. Because the aloe sample has no physical camera calibration, the demonstration depth scale is not a measurement of the real scene.
