import numpy as np


# =========================
# 1. 基本参数
# =========================
# 这个示例展示滑模控制（SMC）的最基本结构：
# - 先定义一个滑模面 s
# - 再设计控制律让系统状态尽快到达 s = 0，并沿着它滑动到原点
dt = 0.005
T = 4.0
g = 9.81
damping = 0.2
disturbance = 0.6
theta_limit = 0.25

# lam 决定滑模面形状；eta 决定到达速度；boundary 用于边界层减小抖振。
lam = 8.0
eta = 4.0
boundary = 0.04

# 真实系统初始状态。
x = 0.05
v = 0.0


def sat(value):
    # 饱和函数替代 sign 函数，形成边界层，减少高频抖振。
    return float(np.clip(value, -1.0, 1.0))


steps = int(T / dt)
for i in range(steps):
    # ---------- 滑模面 ----------
    # 这里选 s = v + lam * x。
    # 当 s -> 0 时，等价于让速度和位置满足一个稳定的一阶关系。
    s = v + lam * x

    # ---------- 滑模控制律 ----------
    # 第一项可看成等效控制，第二项是切换项（这里用饱和函数平滑化）。
    theta_cmd = -(lam * v + eta * sat(s / boundary)) / g
    theta_cmd = float(np.clip(theta_cmd, -theta_limit, theta_limit))

    # ---------- 真实系统 ----------
    a = g * theta_cmd - damping * v + disturbance
    v += a * dt
    x += v * dt

    # 输出滑模面 s，便于观察系统是否逐步逼近滑模面。
    if i % 20 == 0:
        print(
            f"time={i*dt:.2f}s | x={x:.4f} | v={v:.4f} | s={s:.4f} | theta_cmd={theta_cmd:.4f}"
        )
