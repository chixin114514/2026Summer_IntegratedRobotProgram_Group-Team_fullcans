# Task3 六格分拣仿真交接说明

本文面向接手这项工作的下一个 AI。文档描述的是当前已经跑通的实现，不是计划。所有数值都是实测值或代码里的实际常量，路径都能直接打开。

当前状态：六件物块全部分拣成功，两个分类区各 3 个对应颜色物块，最终校验 PASS。旧仿真 `task3` 全程未被改动。

## 1 需求与验收

原始需求：新建一个六格仿真，6 个方格上放 3 个黄色物块和 3 个绿色物块，另设两个分类区，最终黄色区里恰好 3 个黄块、绿色区里恰好 3 个绿块；必须新开一套仿真，不能污染原有的那套。

验收方式有两个层次。

第一层是每个分拣目标的动作结果，`/task3_six/sort_object` 返回的 `success`、`pick_verified`、`place_verified` 都为真。判定逻辑在 `task3_sim/task3_sim/pose_evidence.py`：`pick_verified` 要求物块相对抓取点抬升超过 0.025 m；`placement_is_valid` 要求物块与目标投放位的水平距离不超过 0.045 m、重心 z 落在 0.410 到 0.445 之间、移动速度不超过 0.03 m/s，且位姿样本新鲜度在 0.5 s 以内。

第二层是最终区域校验，用 `tools/verify_regions.py` 读取 `/task3_six/gazebo/pose/info`，判断每个物块是否落在对应区域半边长 0.065 m 的范围内。

## 2 环境与目录

目标主机 `nvidia@192.168.31.146`，ROS 2 Humble，Ignition Fortress，headless 服务器与 GUI 都可运行。从 Mac 侧已配置 SSH 免密，直接 `ssh -o StrictHostKeyChecking=no nvidia@192.168.31.146` 即可，本机没有 `sshpass`。

工作区有三处：

| 用途 | 路径 |
| --- | --- |
| Mac 源码（编辑这里） | `~/overleaf modify/robot_fullcans/task3/ros2_ws/src/` |
| Jetson 部署（编译运行这里） | `~/Desktop/2026Summer_IntegratedRobotProgram_Group-Team_fullcans/task3/ros2_ws/` |
| 标定与运维脚本 | `~/overleaf modify/robot_fullcans/task3/tools/` |

`tools/` 里的脚本已经改成相对自身定位路径，拷到别处也能跑。

新旧两套仿真靠环境变量隔离：`ROS_DOMAIN_ID=107`、`IGN_PARTITION=task3_six_validation_13`。旧的单桌仿真进程名是 `ign gazebo server`，新仿真的命令行里带 `task3_six_world.sdf`，所以清理时用 `pkill -f task3_six` 只会命中新的一套。

## 3 运行流程

改动源码后先同步并编译：

```bash
bash ~/overleaf\ modify/robot_fullcans/task3/tools/deploy.sh
```

它做两件事：把 `ros2_ws/src/` rsync 到 Jetson，然后在 Jetson 上执行 `colcon build --merge-install --packages-select task3_interfaces task3_sim task3_six_sim`。

带上界面运行（Jetson 桌面在 X11 的 :1）：

```bash
# 终端 1
cd ~/Desktop/2026Summer_IntegratedRobotProgram_Group-Team_fullcans/task3/ros2_ws
export ROS_DOMAIN_ID=107 IGN_PARTITION=task3_six_validation_13
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch task3_six_sim task3_six_scene.launch.py headless:=false

# 终端 2，等终端 1 出现 Task3 pick_sort_server ready（约 25 s，机器人 spawn 完成）
cd ~/Desktop/2026Summer_IntegratedRobotProgram_Group-Team_fullcans/task3/ros2_ws
export ROS_DOMAIN_ID=107 IGN_PARTITION=task3_six_validation_13
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run task3_six_sim sort_all
```

无人值守时用 `tools/run_all.sh`（headless）或 `tools/run_gui.sh`（带界面），两者都会先清理旧进程、清空日志、起仿真，再在后台跑分拣并写 `/tmp/sort_all.log`。

只跑其中几格可以限制目标集合，调参时能省很多时间：

```bash
TASK3_SIX_JOBS=P4 ros2 run task3_six_sim sort_all
TASK3_SIX_JOBS=P1,P2 ros2 run task3_six_sim sort_all
```

跑完校验区域：

```bash
python3 tools/verify_regions.py      # 在 Jetson 上运行，且已 source 工作区
```

逐帧事件日志在 `~/.ros/task3_six_sort_log.jsonl`，每个 stage 一行，含 `joint_error_deg`、`trim_deg`、`measured_deg`、`tool_xyz`、`object_xyz`，放置阶段还多一行 `place_correction`。

## 4 系统结构

动作接口在 `task3_interfaces/action/SortObject.action`：目标字段 `grid_id`、`bin_id`；结果字段 `success`、`pick_verified`、`place_verified`、`object_name`、`message`；反馈字段 `stage`、`progress`。服务端是新包里的 `six_pick_sort_server`，它是 `task3_sim` 里 `PickSortServer` 的子类，复用原有的运动规划、位姿订阅与判定逻辑。

一个目标展开成 21 个阶段，顺序和时长键如下（映射来自 `task3_sim/task3_sim/motion_plan.py`）：

| 阶段 | 时长键 | 说明 |
| --- | --- | --- |
| OPEN | gripper | 归位、张开 |
| PICK_APPROACH_1 / 2 | transfer | 折返到目标方位 |
| PICK_ABOVE | transfer | 方格上方 |
| PICK_DESCEND_1 / 2 / 3 | descent_segment | 竖直下降到插入深度 |
| PICK | vertical | 到抓取高度 |
| CLOSE | gripper | 夹紧 |
| LIFT_ASCEND_1 / 2 / 3 | descent_segment | 沿原路抬出 |
| LIFT | descent_segment | 回到上方 |
| TRANSFER_LIFT | vertical | 抬到转移高度 |
| TRANSFER_ROTATE | transfer | 转向投放方位 |
| BIN_ABOVE | vertical | 投放位上方 |
| BIN_PLACE | vertical | 下放到投放高度 |
| RELEASE | gripper | 张开 |
| SETTLE | settle | 由 `_run_settle` 处理，等待落定 |
| RETREAT | vertical | 抬起 |
| HOME | home | 回零 |

每个阶段分两相执行。第一相按 smoothstep 从上一姿态插值到目标姿态，只下发指令；第二相保持目标，闭环补偿在这里积分，同时每 tick 检查关节跟踪误差与关节速度，满足收敛判据就提前退出，否则等到 `hold_timeout_s` 超时。当前配置下一轮全程约 250 s，六件约 25 分钟。

时长配置在 `task3_six_sim/config/motion_points_six.yaml` 的 `motion.durations_s`：`home 2.0`、`transfer 2.5`、`vertical 2.5`、`gripper 1.5`、`settle 1.0`、`descent_segment 4.0`；另有 `command_rate_hz 20`、`settle_velocity_rad_s 0.03`、`hold_timeout_s 10.0`。

## 5 三层闭环补偿

失败的根源是机器人描述的关节位置环太软。`task3_sim/urdf/mecharm_270_gazebo.urdf` 里六个臂关节的插件参数是 `p_gain` 8/8/8/5/5/4、`i_gain` 0.1/0.1/0.1/0.05/0.05/0.05、`d_gain` 0.4/0.4/0.4/0.25/0.25/0.20。实测稳态跟踪误差 3 到 10 度，末端因此偏离目标 20 到 45 mm，比夹爪 42 mm 的开口还大，而且误差随姿态变化，所以不存在一个固定的补偿位姿能救回来。提高 `p_gain` 试过多次（最高到 200），效果更差或引发振荡，已经放弃并把 URDF 还原成原始文件，launch 里的路径指向 `task3_sim` 的原件。

三层补偿全部写在 `task3_six_sim/task3_six_sim/six_pick_sort_server.py`，由一条原则串起来：所有积分型闭环只在阶段第二相工作，第一相斜坡期间一律冻结。原因是被控对象只交付大约三分之一的指令变化量，参考量在动的时候积分器累积的是伺服滞后而不是误差，一定会饱和，然后把末端推得比原来更偏。

### 5.1 关节指令积分补偿

`_trim` 是最基础的一层。阶段第二相里每 tick 计算 `指令角 - 实测角`（指令是度，桥接的 JointState 是弧度，需要换算），按 `joint_trim_gain` 积进下发的指令，再按关节限位留 3 度余量做抗积分饱和钳制。参数为 `joint_trim_gain 2.0`、`joint_trim_limit_deg 40.0`、收敛容差 `joint_error_tolerance_deg 0.2`。

效果：第二相内实测关节角收敛到目标 0.25 度以内，末端落到正运动学预测的位置上，而这个位置正好是物块中心。六个方格全部核对过，末端中点与物块的水平偏差在 0.5 mm 量级。

### 5.2 笛卡尔物体跟随

`_refresh_follow` 用实测物块位置和实测末端垫片中点做三维最小范数修正，自由度选 `(J1, J6)` 配对、`J2`、`J3`、`J5`，配对 J1 与 J6 的目的是让夹爪闭合轴始终保持在世界 Y 方向。参数为 `cartesian_follow_gain` 和 `cartesian_follow_limit_deg 18.0`。

它的积分版本会自激，所以当前配置里 `cartesian_follow_gain` 是 0.0，即关闭。原因是它在斜坡相里被刷新，参考量一直移动，积分器累积滞后并顶到限幅，反而把末端从物块推开 35 mm，比它要消除的误差大一个量级。代码路径保留着，因为排查问题时有用，但它不属于当前方案。

### 5.3 放置闭环

`_place_fix` 是真正决定成败的一层。抓取点在夹爪垫片中点下方约 13 mm，转移中物块还会在爪内滑动几毫米，而且这个伸展姿态下真实垫片中点比正运动学预测低约 10 mm。三者叠加的结果是：按名义投放位下放，物块会被按进桌面，手臂顶死，`_trim` 饱和，物块在夹爪随便所在的位置被放开；或者被从 10 mm 高处丢下，倒伏后滚出区域。这两种失效在日志里都能看到。

处理办法分两步。先在 `BIN_ABOVE` 阶段结束时（此时位姿已收敛）测量物块到投放位的偏差，作为初值转换成关节偏移；然后在 `BIN_PLACE` 和 `RELEASE` 两个阶段的第二相里，把物块实测位置对投放位做积分，xy 与高度同时收敛。高度目标取物块重心 0.4259 m，即静止高度 0.425 上方 0.9 mm，避免掉落。

高度闭环只在 `BIN_PLACE` 和 `RELEASE` 生效（`PLACE_Z_STAGES`），因为若从 `BIN_ABOVE` 就开始把物块拉到静止高度，手臂会在离桌面 100 mm 的高处被一路拽下去。`BIN_ABOVE` 只负责测初值。

相关常量：`PLACE_OBJECT_Z_M 0.4259`、`PLACE_MAX_SHIFT_M 0.05`、`PLACE_LIMIT_DEG 25.0`、`PLACE_GAIN 1.5`、`UPRIGHT_MIN_Z_M 0.415`（物块重心低于此值说明已倒伏，此时停止跟随）。

## 6 场景与标定数据

物块外形 0.040 × 0.018 × 0.050 m，其中 0.018 沿夹爪闭合轴、0.040 沿切向，质量 0.08 kg，摩擦系数 2.0，接触刚度 200000、阻尼 100，放在台面上时重心 z = 0.425。桌面 0.80 × 0.70 × 0.40，台面高度 0.40，机械臂在 (0, 0, 0.40) 处 spawn。场景文件是 `task3_six_sim/worlds/task3_six_world.sdf`，坐标表与物块到方格、投放位的映射在 `task3_six_sim/config/scene_six.yaml`。

六个方格按 r = 0.165 m 的圆弧均匀排布，方位角从 34.25° 起、每 22.15° 一个，相邻方格中心相距 63.8 mm，比物块长度与夹爪宽度都大：

| 方格 | x | y | 方位角 |
| --- | --- | --- | --- |
| P1 | 0.136387 | 0.092863 | 34.25° |
| P2 | 0.091310 | 0.137432 | 56.40° |
| P3 | 0.032755 | 0.161716 | 78.55° |
| P4 | -0.030635 | 0.162131 | 100.70° |
| P5 | -0.089503 | 0.138615 | 122.85° |
| P6 | -0.135160 | 0.094640 | 145.00° |

投放位与分类区：

| 名称 | x | y |
| --- | --- | --- |
| YELLOW_1 | 0.106557 | -0.047883 |
| YELLOW_2 | 0.144057 | -0.026233 |
| YELLOW_3 | 0.144057 | -0.069533 |
| GREEN_1 | -0.072883 | -0.131557 |
| GREEN_2 | -0.035383 | -0.109907 |
| GREEN_3 | -0.035383 | -0.153207 |
| YELLOW 区 | 0.131557 | -0.047883 |
| GREEN 区 | -0.047883 | -0.131557 |

两区半边长都是 0.065 m，每个区里放三个投放位，彼此相距 43 mm。

物块到目标的默认配对为 P1→YELLOW_1、P2→GREEN_1、P3→YELLOW_2、P4→GREEN_2、P5→YELLOW_3、P6→GREEN_3，写在 `task3_six_sim/sort_all_client.py` 的 `SORT_JOBS` 里。

夹爪几何：两个橡胶垫是 `gripper_left1` 与 `gripper_right1`，垫片尺寸 0.006 × 0.030 × 0.034，相对连杆的局部偏置是 ±0.020、0、0.022，所以计算末端位置时必须把局部偏置按连杆姿态旋转后加上，只取连杆原点会差 2 cm。张开量 `open_rad 0.14` 对应垫片间距 41.9 mm，闭合量 `closed_rad -0.65`。垫片张开时向外张约 8 度，下缘实际间隙只剩几毫米，这就是几毫米的横向误差就能挂倒物块的原因。

关节限位（度）：下限 `[-160, -75, -175, -155, -115, -180]`，上限 `[160, 120, 65, 155, 115, 180]`。补偿量在 `_publish_arm` 与两个积分器里都会按这组限位、留 3 度余量钳制，否则补偿会把关节推过限位，手臂摆到无关位置。

运动位姿的求解工具在 `tools/solve_picks.py`，它用与运行时同一个 `kinematics.py` 做正运动学，让夹爪中点精确落在物块中心，五个高度层级只用 z 区分，保证下降是纯竖直的：

| 层级 | 垫片中点 z |
| --- | --- |
| pick | 0.426 |
| descent_3 | 0.447 |
| descent_2 | 0.477 |
| descent_1 | 0.541 |
| above | 0.576 |

投放位是 place 0.432、above 0.532。工具把结果写到脚本旁边的 `motion_ik.yaml` 供比对，确认无误后覆盖 `task3_six_sim/config/motion_points_six.yaml`。重跑一遍能复现当前部署的全部姿势：picks 完全一致，bins 只有 1e-4 度量级的迭代噪声。

## 7 工具清单

`tools/` 目录下：

| 脚本 | 作用 |
| --- | --- |
| `solve_picks.py` | 按方格坐标解算抓取与投放位姿，生成 `motion_points_six.yaml` |
| `respace.py` | 重排六个方格的极坐标布局，同时改写 world 与 scene 配置（先输出到脚本旁供比对） |
| `verify_regions.py` | 跑完后校验两个区域里的物块数量与颜色 |
| `fkcheck.py` | 把 `kinematics.py` 的预测与仿真里实测的垫片位置做对比 |
| `hold_pose.py`、`measure_pose.sh` | 把手臂固定在某个关节向量上并等待稳定，用来单独观测某个位姿的跟随误差 |
| `deploy.sh` | 同步源码到 Jetson 并编译 |
| `run_all.sh`、`run_gui.sh` | 一键起仿真并跑完整分拣，前者 headless，后者带界面 |

## 8 实测结果

最终区域校验输出：

```
object                     x         y        z   verdict
orange_battery_1    +0.09814  -0.06554   0.4140   IN yellow region
orange_battery_2    +0.13954  -0.01090   0.4312   IN yellow region
orange_battery_3    +0.14261  -0.06684   0.4300   IN yellow region
green_can_1         -0.08157  -0.11656   0.4300   IN green region
green_can_2         -0.01184  -0.09823   0.4312   IN green region
green_can_3         -0.04096  -0.10957   0.4315   IN green region

yellow blocks in YELLOW region: 3/3
green  blocks in GREEN  region: 3/3
RESULT: PASS
```

同一轮里六件全部 `pick_verified=true`、`place_verified=true`，无失败。抓取阶段实测末端中点与物块的水平偏差为：`PICK_DESCEND_2` 约 0.3 mm，`PICK_DESCEND_3` 约 0.1 mm，即夹爪张开状态下物块基本处于正中。

`kinematics.py` 与仿真的对照结果：水平方向偏差小于 0.5 mm，竖直方向真实值比预测低 2 到 11 mm，量随伸展程度变化。这个下垂量是放置闭环存在的部分理由。

单元测试：`task3_six_sim/test/test_six_sim.py` 六项全过；`task3_sim/test/` 下 52 项全过，其中 `test_motion_server_contract.py` 用来确认旧包的行为契约没有被破坏。

## 9 已知问题

`PICK` 阶段会把物块顶住，关节补偿一路积到 40 度限幅，收敛判据永远不满足，于是每次都要耗满 10 s 超时，六件下来浪费一分钟左右，还会让下一阶段的补偿起点偏离。它不影响结果，但性价比不高，可以考虑删掉这一步或者缩短它的 hold。

`TRANSFER_ROTATE` 是全程最大的一次运动，P4 到 GREEN_2 时 J1 要转约 208 度，斜坡相补偿冻结，滞后能到 17 度，靠后面的 `BIN_ABOVE` 恢复。想稳一点可以加长 `transfer`，或者把转移拆成两段。

物块在爪内会滑动，本轮实测滑动量从 1 mm 到 20 mm 不等，重复性不好。目前靠放置闭环兜住，但极端情况下物块会倒。倒伏后重心落在 0.409 到 0.414 之间，而 `placement_is_valid` 的下限是 0.410，所以有时能过有时不能。如果要求六个物块全部直立，需要把释放高度再压低，或者在 `CLOSE` 之后增加一段让夹爪二次收紧的动作。

## 10 硬约束

下面几条是踩过的坑，改动前请确认不会破坏它们。

所有积分型闭环必须在阶段第一相保持冻结。被控对象的增益只有标称值的三分之一左右，参考量移动时积分器累积的是滞后，一定会饱和并把末端推离目标。当前实现里 `_trim`、`_follow`、`_place_fix` 三者一致遵守这条。

不要用 `pkill -f 'ign gazebo'` 清理进程，那会杀掉旧仿真，用 `pkill -f task3_six`。

`task3_sim` 除了早先加入的向后兼容改动（`pick_sort_server.py` 里新增的 `action_name` 参数、`motion_plan.py` 里可选的 `press` 步骤），其余部分不要动。它的 URDF、`config/motion_points.yaml`、`config/scene.yaml`、`worlds/task3_world.sdf` 都是旧仿真的组成部分，本轮全程保持原始内容，md5 与源码一致。新逻辑一律放进 `task3_six_sim`。

`i_gain` 是 0.1 和 0.05，对应 80 s 量级的积分时间常数。如果单独把一个位姿按住不放手，几秒钟后误差看起来没消，但放几十秒它自己会到位。这是被控对象自身的积分作用，不是补偿没生效，排查时不要被这一点误导。

脚本里不要加 `set -u`，ROS 的 `setup.bash` 在未绑定的 `AMENT_TRACE_SETUP_FILES` 上会报错退出。

带界面运行时需要 `DISPLAY=:1` 和 `XAUTHORITY=/run/user/1000/gdm/Xauthority`。启动日志里的 `libEGL ... failed to create dri2 screen` 是 Jetson 上的 EGL 加载告警，不影响仿真，必要时可以先 `export LIBGL_ALWAYS_SOFTWARE=1`。

从 Mac 侧写入这个挂载目录时，python 直接新建文件可能被沙箱拒绝，而 `mkdir` 和 `cp` 可以。生成新文件时先写到临时目录再拷过去。
