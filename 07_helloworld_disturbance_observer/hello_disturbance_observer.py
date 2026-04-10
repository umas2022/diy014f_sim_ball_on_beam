import numpy as np


# =========================
# 1. 基本参数
# =========================
# 扰动观测器（DOB）的核心思路是：
# - 先按名义模型设计一个普通控制器
# - 再估计“真实系统和名义模型之间差了多少”
# - 最后把这个差额作为扰动补偿掉
dt = 0.005
T = 5.0
g = 9.81
damping = 0.2
disturbance = 0.8
theta_limit = 0.25

# 名义控制器使用一个简单 PD。
kp = 18.0
kd = 7.0

# 扰动观测器收敛速度。
dob_gain = 30.0

# 真实系统状态。
x = 0.05
v = 0.0

# 扰动估计值，初始先设为 0。
d_hat = 0.0


steps = int(T / dt)
for i in range(steps):
    # ---------- 名义控制器 ----------
    # 假设名义模型是 x_ddot = g * theta，不考虑阻尼和外扰。
    theta_pd = (-kp * x - kd * v) / g

    # ---------- 扰动补偿 ----------
    # 如果 d_hat 估计得准，那么减掉 d_hat / g 就能抵消大部分扰动。
    theta_cmd = theta_pd - d_hat / g
    theta_cmd = float(np.clip(theta_cmd, -theta_limit, theta_limit))

    # ---------- 真实系统 ----------
    # 真实系统比名义模型多了阻尼和外部扰动。
    a = g * theta_cmd - damping * v + disturbance
    v += a * dt
    x += v * dt

    # ---------- 扰动观测器 ----------
    # 名义模型认为的加速度。
    nominal_a = g * theta_cmd

    # 真实加速度与名义加速度的差，就是“未建模部分”的加速度误差。
    accel_error = a - nominal_a

    # 一阶滤波形式的扰动估计更新。
    d_hat += dob_gain * (accel_error - d_hat) * dt

    # 观察位置、速度和扰动估计值的变化。
    if i % 20 == 0:
        print(
            f"time={i*dt:.2f}s | x={x:.4f} | v={v:.4f} | "
            f"theta_cmd={theta_cmd:.4f} | d_hat={d_hat:.4f}"
        )
