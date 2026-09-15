"""把夹爪两片橡胶垫的中点送到指定 XYZ 的逆解，只动 J2/J3/J5。

J1 / J4 / J6 固定不参与解算：
    J1 = 目标点的方位角 atan2(Y, X)，也就是"把关节 1 转到对应位置"；
    J4 = 固定值（默认 0）；
    J6 = J1，这样两个旋转互相抵消，夹爪的闭合方向始终平行于世界坐标轴，
         张开的两片橡胶垫才能对称地跨在物体两侧。

关于解算方式（这里踩过一个坑，记下来）：
    J4 固定时，三个关节对夹爪中点的三个偏导列**全部落在机械臂的运动平面内**，
    所以这个 3x3 雅可比是恒秩 2 的（实测 det ≈ -8e-7）。
    如果用常规的 ``(J^T J + λI) Δq = J^T e``，法方程本身就是奇异的，解出来的
    步长会被零空间放大到几十弧度（实测 84 rad），截断之后每一步都走在无用的
    方向上，永远收敛不了。
    所以这里用**最小范数阻尼最小二乘**：
        Δq = J^T (J J^T + λ² I)^{-1} e
    这个 3x3 只依赖 J J^T，秩亏时也不会爆；因为误差 e 落在 J 的行空间里，
    零空间方向不会被激励。再叠加一个零空间姿态偏置，避免关节沿零空间漂走。

正运动学直接用 task3_six_sim.kinematics（和仿真里标定过的模型是同一份）。
纯 Python，不需要 numpy。
"""

from __future__ import annotations

import math

from task3_six_sim import kinematics

# 有限差分求雅可比时的关节扰动（度）
_JACOBIAN_STEP_DEG = 0.5
# 收敛判据：残差小于这个值（米）就认为到了
CONVERGED_M = 1e-4
# 阻尼系数的相对量级（乘在 max(diag(J J^T)) 上）
_REGULARIZATION_RELATIVE = 1e-8
# 零空间姿态偏置增益。只作用在末端控制不了的那个自由度上，
# 所以放大它不会影响末端精度，只会更快把关节拉回偏好构型。
_POSTURE_GAIN = 0.30
# 没传 posture 时用的默认偏好构型（J2, J3, J5），
# 取自仿真里标定过的抓取分支中间位置。绝对不能用 seed_q 兜底。
POSTURE_FALLBACK = (30.0, -17.0, 65.0)


def _solve3(a, b):
    """3x3 高斯消元（列主元）。退化返回 None。"""
    m = [list(a[i]) + [float(b[i])] for i in range(3)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(m[row][col]))
        if abs(m[pivot][col]) < 1e-15:
            return None
        m[col], m[pivot] = m[pivot], m[col]
        for row in range(col + 1, 3):
            factor = m[row][col] / m[col][col]
            for k in range(col, 4):
                m[row][k] -= factor * m[col][k]
    out = [0.0, 0.0, 0.0]
    for row in (2, 1, 0):
        total = m[row][3] - sum(m[row][k] * out[k] for k in range(row + 1, 3))
        out[row] = total / m[row][row]
    return out


def assemble(j1_deg, q, j4_deg):
    """把 (J1, J2, J3, J4, J5, J6) 拼成完整六个关节角。"""
    return [float(j1_deg), float(q[0]), float(q[1]),
            float(j4_deg), float(q[2]), float(j1_deg)]


def pad_mid(j1_deg, q, j4_deg=0.0, grip_rad=0.14, base_z=0.0):
    """夹爪中点（两片橡胶垫的中点）在世界坐标里的 XYZ。"""
    return kinematics.pad_mid(assemble(j1_deg, q, j4_deg), grip_rad, base_z)


def _jacobian(j1_deg, q, j4_deg, grip_rad, base_z, mid):
    """数值雅可比 d(世界坐标) / d(弧度)，3x3。"""
    out = [[0.0] * 3 for _ in range(3)]
    for column in range(3):
        perturbed = list(q)
        perturbed[column] += _JACOBIAN_STEP_DEG
        moved = pad_mid(j1_deg, perturbed, j4_deg, grip_rad, base_z)
        for axis in range(3):
            out[axis][column] = (
                (moved[axis] - mid[axis]) / math.radians(_JACOBIAN_STEP_DEG)
            )
    return out


def solve(target_xyz, seed_q, j1_deg, j4_deg=0.0, grip_rad=0.14, base_z=0.0,
          iterations=400, max_step_deg=4.0, posture=None, limits=None):
    """解出 (J2, J3, J5)，返回 ``((J2, J3, J5), 残差米)``。

    残差 > ``CONVERGED_M`` 说明这个点够不到或落在奇异构型附近；调用方只记录
    日志，不中止（按用户要求不做失败退出）。

    ``seed_q`` 传上一步的解，让解沿着同一条分支走，相邻目标之间不突变。

    ``posture`` 是零空间里要优先保持的姿态，**必须传一个固定的值**（比如配置
    里那组种子构型）。如果传成上一步的解，``drift = posture - q`` 在第一次迭代
    时恒为 0，偏置项等于没写，关节就会沿着那个没人管的自由度一路漂出去
    （实测 J5 从 51° 漂到 243°）。所以这里的默认值是 ``POSTURE_FALLBACK``，
    而不是 ``seed_q``。

    ``limits`` 是 (J2, J3, J5) 的限位 ``(low, high)``，**在迭代内部就夹住**。
    只在解完之后截断是没用的：截断会改变末端位置，解出来的位姿就对不上目标了。
    """
    tx, ty, tz = (float(value) for value in target_xyz)

    if limits:
        low = [float(value) for value in limits[0]]
        high = [float(value) for value in limits[1]]
    else:
        low, high = [-1e9, -1e9, -1e9], [1e9, 1e9, 1e9]

    def feasible(values):
        return [min(high[i], max(low[i], float(values[i]))) for i in range(3)]

    q = feasible(seed_q or POSTURE_FALLBACK)
    preferred = feasible(posture or POSTURE_FALLBACK)
    best_q = list(q)
    best_residual = float("inf")
    step_limit = math.radians(max(0.1, float(max_step_deg)))

    for _ in range(max(1, int(iterations))):
        mid = pad_mid(j1_deg, q, j4_deg, grip_rad, base_z)
        error = [tx - mid[0], ty - mid[1], tz - mid[2]]
        residual = math.sqrt(sum(value * value for value in error))
        if residual < best_residual:
            best_q = list(q)
            best_residual = residual
        if residual < 1e-9:
            break

        jac = _jacobian(j1_deg, q, j4_deg, grip_rad, base_z, mid)

        # J J^T（3x3）
        gram = [[sum(jac[i][k] * jac[j][k] for k in range(3)) for j in range(3)]
                for i in range(3)]
        scale = max(abs(gram[i][i]) for i in range(3))
        damping = _REGULARIZATION_RELATIVE * max(scale, 1e-12)
        for index in range(3):
            gram[index][index] += damping

        multiplier = _solve3(gram, error)          # y = (J J^T + λ²I)^{-1} e
        if multiplier is None or not all(math.isfinite(v) for v in multiplier):
            break

        # Δq_task = J^T y
        step = [sum(jac[k][i] * multiplier[k] for k in range(3)) for i in range(3)]

        # 零空间姿态偏置：把 (I - J^+ J) 作用在"回到偏好姿态"的方向上，
        # 只动末端本来也控制不了的那个自由度，不影响末端精度。
        drift = [math.radians(preferred[i] - q[i]) * _POSTURE_GAIN for i in range(3)]
        jdrift = [sum(jac[i][k] * drift[k] for k in range(3)) for i in range(3)]
        projected = _solve3(gram, jdrift)
        if projected is not None and all(math.isfinite(v) for v in projected):
            null_part = [
                drift[i] - sum(jac[k][i] * projected[k] for k in range(3))
                for i in range(3)
            ]
            step = [step[i] + null_part[i] for i in range(3)]

        if not all(math.isfinite(value) for value in step):
            break
        length = math.sqrt(sum(value * value for value in step))
        if length > step_limit:
            step = [value * step_limit / length for value in step]
        # 迭代内部就夹到限位内，保证返回的解一定是可达的
        q = feasible([q[index] + math.degrees(step[index]) for index in range(3)])

    return tuple(best_q), best_residual
