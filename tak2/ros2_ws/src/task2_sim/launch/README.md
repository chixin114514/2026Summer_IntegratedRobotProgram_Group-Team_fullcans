# Launch files

## Formal entry point

Use only:

```bash
ros2 launch task2_sim task2.launch.py
```

for the submitted Task 2 simulation/real-robot acceptance workflow.

## Legacy / tuning launch files

The following files are retained only for historical experiments or tuning and are **not** part of the formal control chain:

- `task2_sim.launch.py`
- `task2_auto.launch.py`
- `task2_tune.launch.py`

Likewise, `joint_controller.py`, `auto_pick_place.py`, and `joint_tuner.py` are legacy/tuning utilities and must not be used for acceptance runs. Their historical HOME values are not authoritative. The formal HOME source is `config/task_points.yaml -> home.joints_deg`.
