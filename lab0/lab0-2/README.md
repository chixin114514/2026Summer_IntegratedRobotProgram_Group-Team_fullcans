# Lab 0-2 Experiment 3 Stereo Depth Estimation

This experiment uses OpenCV's official `aloeL.jpg` and `aloeR.jpg` stereo sample pair. The pair is already rectified, so `stereo_depth_estimation.py` computes horizontal-line correspondences with StereoSGBM, converts disparity to depth with `Z=fB/d`, and writes a colored PLY point cloud.

Run from this directory:

```bash
cd lab0/lab0-2
python3 code/stereo_depth_estimation.py \
  --left data/aloeL.jpg \
  --right data/aloeR.jpg \
  --output-dir results
```

The English report is in `report/`. The tested run produced a 70.79% valid-disparity ratio and 249,334 finite point-cloud vertices. The default focal length and baseline are demonstration values; replace them with real stereo-calibration values for metric depth.
