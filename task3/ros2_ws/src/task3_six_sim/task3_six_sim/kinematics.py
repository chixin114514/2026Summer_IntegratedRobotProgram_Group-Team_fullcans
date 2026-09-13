"""Forward kinematics and a small XY Jacobian for the six-grid gripper.

Mirrors the joint origins/axes of ``task3_sim/urdf/mecharm_270_gazebo.urdf``.
Only the tool (the midpoint of the two rubber pads) is needed, so the frames are
propagated down to ``gripper_left1`` / ``gripper_right1`` and no further.

Pure Python on purpose: the action server should not need numpy at runtime.
"""

from __future__ import annotations

import math

# name: (parent, child, origin, rpy, axis)  -- axis None means fixed
JOINTS = {
    "joint1_to_base": ("base", "link1", (0.0, 0.0, 0.1), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    "joint2_to_joint1": ("link1", "link2", (0.0, 0.0, 0.038), (-1.5708, 0.0, 0.0), (0.0, 0.0, 1.0)),
    "joint3_to_joint2": ("link2", "link3", (0.0, -0.1, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    "joint4_to_joint3": ("link3", "link4", (0.108, -0.005, -0.001), (0.0, 1.5708, 0.0), (0.0, 0.0, 1.0)),
    "joint5_to_joint4": ("link4", "link5", (-0.001, 0.0, 0.0), (0.0, -1.5708, 0.0), (0.0, 0.0, 1.0)),
    "joint6_to_joint5": ("link5", "link6", (0.06, 0.0, 0.0), (0.0, 1.5708, 0.0), (0.0, 0.0, 1.0)),
    "gripper_mount": ("link6", "gripper_palm", (0.0, 0.0, 0.005), (0.0, 0.0, 0.0), None),
    "gripper_controller": ("gripper_palm", "gripper_left3", (-0.012, 0.0, 0.005), (0.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
    "gripper_left3_to_gripper_left1": ("gripper_left3", "gripper_left1", (-0.027, 0.0, 0.016), (0.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
    "gripper_base_to_gripper_left2": ("gripper_palm", "gripper_left2", (-0.005, 0.0, 0.027), (0.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
    "gripper_base_to_gripper_right3": ("gripper_palm", "gripper_right3", (0.012, 0.0, 0.005), (0.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
    "gripper_right3_to_gripper_right1": ("gripper_right3", "gripper_right1", (0.027, 0.0, 0.016), (0.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
    "gripper_base_to_gripper_right2": ("gripper_palm", "gripper_right2", (0.005, 0.0, 0.027), (0.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
}

ORDER = ["joint1_to_base", "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
         "joint5_to_joint4", "joint6_to_joint5", "gripper_mount", "gripper_controller",
         "gripper_left3_to_gripper_left1", "gripper_base_to_gripper_left2",
         "gripper_base_to_gripper_right3", "gripper_right3_to_gripper_right1",
         "gripper_base_to_gripper_right2"]

PAD_LOCAL = {"gripper_left1": (0.020, 0.0, 0.022), "gripper_right1": (-0.020, 0.0, 0.022)}

# The tool centre only moves in XY under J2/J3/J5 once J1/J4/J6 are fixed, but
# those three all push it along the same radial direction, so a tangential
# correction needs the base swing as well.  J1 is therefore moved together with
# J6 so the jaws keep closing along world Y (the two rotations cancel).
XY_DOFS = ((0, 5), (1,), (2,), (4,))


def _rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _axis_rot(axis, q):
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    ax, ay, az = ax / n, ay / n, az / n
    c, s = math.cos(q), math.sin(q)
    C = 1.0 - c
    return [
        [c + ax * ax * C, ax * ay * C - az * s, ax * az * C + ay * s],
        [ay * ax * C + az * s, c + ay * ay * C, ay * az * C - ax * s],
        [az * ax * C - ay * s, az * ay * C + ax * s, c + az * az * C],
    ]


def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mv(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def pad_centres(arm_deg, grip_rad, base_z=0.40):
    """World XYZ of both rubber pad centres for a set of arm/gripper commands."""
    q = {name: math.radians(float(value)) for name, value in zip(ORDER[:6], arm_deg)}
    v = float(grip_rad)
    q["gripper_mount"] = 0.0
    q["gripper_controller"] = v
    q["gripper_left3_to_gripper_left1"] = -v
    q["gripper_base_to_gripper_left2"] = v
    q["gripper_base_to_gripper_right3"] = -v
    q["gripper_right3_to_gripper_right1"] = v
    q["gripper_base_to_gripper_right2"] = -v

    R = {None: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]}
    t = {None: [0.0, 0.0, base_z]}
    R["world"], t["world"] = R[None], t[None]
    placed = {None, "world"}
    for name in ORDER:
        parent, child, origin, rpy, axis = JOINTS[name]
        while parent not in placed:
            R[parent], t[parent] = R[None], t[None]
            placed.add(parent)
        rotation = _rpy_to_R(*rpy)
        if axis is not None:
            rotation = _mm(rotation, _axis_rot(axis, q[name]))
        offset = _mv(R[parent], list(origin))
        R[child] = _mm(R[parent], rotation)
        t[child] = [t[parent][i] + offset[i] for i in range(3)]
        placed.add(child)

    out = {}
    for link, local in PAD_LOCAL.items():
        out[link] = [t[link][i] + _mv(R[link], list(local))[i] for i in range(3)]
    return out["gripper_left1"], out["gripper_right1"]


def pad_mid(arm_deg, grip_rad, base_z=0.40):
    left, right = pad_centres(arm_deg, grip_rad, base_z)
    return [(left[i] + right[i]) / 2.0 for i in range(3)]


def _solve3(matrix, vector):
    """Solve a 3x3 system by Gaussian elimination with partial pivoting."""
    a = [list(matrix[i]) + [float(vector[i])] for i in range(3)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(a[row][column]))
        if abs(a[pivot][column]) < 1e-15:
            return None
        a[column], a[pivot] = a[pivot], a[column]
        for row in range(column + 1, 3):
            factor = a[row][column] / a[column][column]
            for k in range(column, 4):
                a[row][k] -= factor * a[column][k]
    out = [0.0, 0.0, 0.0]
    for row in (2, 1, 0):
        total = a[row][3] - sum(a[row][k] * out[k] for k in range(row + 1, 3))
        out[row] = total / a[row][row]
    return out


def xyz_correction(arm_deg, grip_rad, error_xyz, gain=0.8, limit_deg=10.0,
                   base_z=0.40, dofs=XY_DOFS):
    """Joint deltas (degrees) that move the tool centre by ``error_xyz`` (metres).

    Three equations (the desired XYZ displacement of the pad midpoint) against
    four dofs, so the least-norm solution is used: it keeps the height and the
    closing axis as close to nominal as possible while still placing the jaws.
    """
    base = [float(value) for value in arm_deg]
    mid0 = pad_mid(base, grip_rad, base_z)
    count = len(dofs)
    jac = [[0.0] * count for _ in range(3)]
    step_deg = 1.0
    for k, members in enumerate(dofs):
        pert = list(base)
        for index in members:
            pert[index] += step_deg
        moved = pad_mid(pert, grip_rad, base_z)
        for axis in range(3):
            jac[axis][k] = (moved[axis] - mid0[axis]) / math.radians(step_deg)

    gram = [[sum(jac[i][k] * jac[j][k] for k in range(count)) for j in range(3)]
            for i in range(3)]
    desired = [float(error_xyz[i]) * gain for i in range(3)]
    y = _solve3(gram, desired)
    if y is None:
        return [0.0] * 6
    deltas = [0.0] * 6
    for k, members in enumerate(dofs):
        value = math.degrees(sum(jac[axis][k] * y[axis] for axis in range(3)))
        value = max(-limit_deg, min(limit_deg, value))
        for index in members:
            deltas[index] = value
    return deltas
