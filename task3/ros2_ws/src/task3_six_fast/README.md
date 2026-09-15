# task3_six_fast 视觉颜色分拣

本包在原六格快速仿真上增加了真正读取相机图像的颜色识别。物块、六个抓取
位置、机械臂运动点和两个收集区域均未改变。

流程如下：

1. 订阅 `/task3_six/camera/image_raw` (`sensor_msgs/Image`)；
2. 根据顶置相机参数，把 P1--P6 的固定桌面坐标投影成图像 ROI；
3. 在每个 ROI 内用 HSV 色相统计判定 `YELLOW` 或 `GREEN`，连续 3 帧一致才确认；
4. 检查结果恰好为 3 黄、3 绿；
5. 给每种颜色依次分配 `YELLOW_1..3` 或 `GREEN_1..3`，调用现有
   `/task3_six/sort_object` Action 完成抓放。

识别不完整、颜色数量不对或图像格式不支持时，节点会安全退出，不移动机械臂。

一条命令启动仿真并自动视觉分拣：

```bash
ros2 launch task3_six_fast task3_six_fast_scene.launch.py \
  headless:=false auto_sort:=true
```

也可以先启动仿真服务，再单独运行识别客户端：

```bash
ros2 run task3_six_fast vision_sort
```

识别日志会显示每格黄/绿像素计数、稳定后的类别，以及由视觉结果生成的
`P? -> COLOR_?` 分拣计划。若相机画面有少量整体偏移，可用参数校正 ROI：

```bash
ros2 run task3_six_fast vision_sort --ros-args \
  -p pixel_offset_u:=2.0 -p pixel_offset_v:=-1.0
```
