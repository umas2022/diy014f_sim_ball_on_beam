import numpy as np


# =========================
# 1. 基本参数
# =========================
# 这个例子展示 Backstepping 的入门思路：
# - 先把速度 v 当成位置子系统的“虚拟控制”
# - 先为 x 设计一个希望 v 去跟踪的虚拟控制 alpha
# - 再进一步设计真实控制 theta_cmd，让 v 跟住 alpha
dt = 0.01
T = 5.0
g = 9.81
damping = 0.15
theta_limit = 0.25

# Backstepping 设计增益。
c1 = 4.0
c2 = 5.0

# 真实系统初始状态。
x = 0.05
v = 0.0


steps = int(T / dt)
for i in range(steps):
    # ---------- 第一步：虚拟控制 ----------
    # 把 v 看成 x 子系统的输入，希望 v 去跟踪 alpha = -c1 * x。
    # 如果 v 真能等于 alpha，那么 x_dot = -c1 * x，就会指数收敛到 0。
    alpha = -c1 * x

    # z2 是真实速度和虚拟控制之间的误差。
    z2 = v - alpha

    # ---------- 第二步：真实控制 ----------
    # 现在再设计 theta_cmd，让 z2 也收敛。
    # 这样就形成“先稳定外层，再逐层往回设计”的 Backstepping 结构。
    theta_cmd = (-x - c2 * z2 - c1 * v + damping * v) / g
    theta_cmd = float(np.clip(theta_cmd, -theta_limit, theta_limit))

    # ---------- 真实系统 ----------
    a = g * theta_cmd - damping * v
    v += a * dt
    x += v * dt

    # 打印 alpha 和 theta_cmd，方便观察虚拟控制与真实控制的关系。
    if i % 10 == 0:
        print(
            f"time={i*dt:.2f}s | x={x:.4f} | v={v:.4f} | alpha={alpha:.4f} | theta_cmd={theta_cmd:.4f}"
        )
