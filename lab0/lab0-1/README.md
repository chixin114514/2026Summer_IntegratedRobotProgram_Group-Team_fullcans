# Lab 0-1 Experiment 2 Smartphone Camera Calibration

This experiment estimates smartphone-camera intrinsic and distortion parameters from checkerboard photographs using OpenCV. The English report is in `report/`. The complete calibration script and usage notes are in `code/`; the selected 30-image dataset is in `dataset/`; and the final YAML, JSON, error report, and corner montage are in `calibration_results/` and `assets/`.

The final calibration used the consistent 960×1280 image group. Fourteen of 16 candidate images were detected successfully (87.50% detection rate), and 12 views were retained for the final calibration after outlier rejection.
