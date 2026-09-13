"""像素坐标 -> 桌面平面坐标的单应矩阵求解与使用。

相机固定在高处俯视，所有物体都落在同一张桌面上，所以"图像点 -> 桌面点"
就是一个平面单应变换（3x3 矩阵）：只要给 4 个以上"像素 ↔ 桌面 XY"对应点
就能解出来。

求解和使用都是纯 Python，动作节点不需要 numpy（和 task3_six_sim 的
正运动学保持同一个风格）。

坐标约定（重要）：
    桌面坐标系原点 = 机械臂底座中心
    +X = J1 = 0 时手臂伸出的方向
    +Y = 从 +X 逆时针 90°（也就是底座的左手边）
    +Z = 向上
    单位：米
"""

from __future__ import annotations

import math


def _solve(a, b):
    """高斯消元（列主元）。a 是 n x n，b 是长度 n，返回解向量；退化返回 None。"""
    n = len(b)
    m = [list(a[i]) + [float(b[i])] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(m[row][col]))
        if abs(m[pivot][col]) < 1e-14:
            return None
        m[col], m[pivot] = m[pivot], m[col]
        for row in range(col + 1, n):
            factor = m[row][col] / m[col][col]
            for k in range(col, n + 1):
                m[row][k] -= factor * m[col][k]
    out = [0.0] * n
    for row in range(n - 1, -1, -1):
        total = m[row][n] - sum(m[row][k] * out[k] for k in range(row + 1, n))
        out[row] = total / m[row][row]
    return out


def solve_homography(pairs, image_size=(640, 480)):
    """由对应点求单应矩阵。

    ``pairs`` 形如 ``[((u, v), (X, Y)), ...]``，``u, v`` 是 YOLO 给的原始像素，
    ``X, Y`` 是桌面坐标（米）。至少 4 组；多于 4 组时按最小二乘拟合。

    像素是几百量级、桌面坐标是零点几米量级，直接解会让法方程病态，所以先把
    像素归一化到图像尺寸再解，最后把缩放折回矩阵，对外仍然是
    "原始像素 -> 米"。返回 3x3（行优先）列表；点不足或退化返回 None。
    """
    if pairs is None or len(pairs) < 4:
        return None
    width = float(image_size[0]) or 640.0
    height = float(image_size[1]) or 480.0

    # 用 h33 = 1 把 8 个未知数写成线性最小二乘：
    #   un*h0 + vn*h1 + h2 - X*un*h6 - X*vn*h7 = X
    #   un*h3 + vn*h4 + h5 - Y*un*h6 - Y*vn*h7 = Y
    rows = []
    rhs = []
    for (u, v), (x, y) in pairs:
        un = float(u) / width
        vn = float(v) / height
        x = float(x)
        y = float(y)
        rows.append([un, vn, 1.0, 0.0, 0.0, 0.0, -x * un, -x * vn])
        rhs.append(x)
        rows.append([0.0, 0.0, 0.0, un, vn, 1.0, -y * un, -y * vn])
        rhs.append(y)

    count = len(rows)
    normal = [[sum(rows[k][i] * rows[k][j] for k in range(count)) for j in range(8)]
              for i in range(8)]
    target = [sum(rows[k][i] * rhs[k] for k in range(count)) for i in range(8)]
    solution = _solve(normal, target)
    if solution is None:
        return None

    normalized = [
        [solution[0], solution[1], solution[2]],
        [solution[3], solution[4], solution[5]],
        [solution[6], solution[7], 1.0],
    ]
    # 把归一化像素折回原始像素：H = Hn @ diag(1/width, 1/height, 1)
    scale = [1.0 / width, 1.0 / height, 1.0]
    return [[normalized[i][j] * scale[j] for j in range(3)] for i in range(3)]


def project(matrix, u, v):
    """原始像素 ``(u, v)`` -> 桌面 ``(X, Y)``（米）。矩阵非法返回 None。"""
    if matrix is None:
        return None
    u = float(u)
    v = float(v)
    denominator = matrix[2][0] * u + matrix[2][1] * v + matrix[2][2]
    if abs(denominator) < 1e-12:
        return None
    x = (matrix[0][0] * u + matrix[0][1] * v + matrix[0][2]) / denominator
    y = (matrix[1][0] * u + matrix[1][1] * v + matrix[1][2]) / denominator
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return x, y


def reprojection_errors(matrix, pairs):
    """每个标定点的重投影误差（米），用来检查标定质量。"""
    out = []
    for (u, v), (x, y) in pairs:
        point = project(matrix, u, v)
        if point is None:
            out.append(float("inf"))
        else:
            out.append(math.hypot(point[0] - float(x), point[1] - float(y)))
    return out
