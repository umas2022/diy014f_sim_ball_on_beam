import numpy as np


# =========================
# 1. 基本参数
# =========================
# 这个脚本的重点不是控制器，而是状态估计：
# 已知只有位置测量 x_meas，尝试同时估计位置 x_hat 和速度 v_hat。
dt = 0.01
T = 6.0

# 真实系统：x_dot = v, v_dot = a_cmd + disturbance
# disturbance 和 process_sigma 用来表示“模型和真实世界不完全一致”。
disturbance = 0.15
measurement_sigma = 0.01
process_sigma = 0.02


# =========================
# 2. Kalman Filter 所需离散模型
#    状态: [x, v]
#    输入: a_cmd
#    测量: z = x
# =========================
# 这里采用最经典的常加速度离散模型。
F = np.array([
    [1.0, dt],
    [0.0, 1.0],
], dtype=float)

B = np.array([
    [0.5 * dt * dt],
    [dt],
], dtype=float)

# 传感器只能测到位置，因此 H 只取状态向量中的第一维。
H = np.array([[1.0, 0.0]], dtype=float)

# Q: 过程噪声协方差，表示对模型不确定性的描述。
# R: 测量噪声协方差，表示对传感器噪声强度的描述。
Q = np.array([
    [1e-5, 0.0],
    [0.0, process_sigma ** 2],
], dtype=float)
R = np.array([[measurement_sigma ** 2]], dtype=float)

rng = np.random.default_rng(7)

# =========================
# 3. 真值状态与滤波器状态
# =========================
# 真实系统状态：仿真器内部持有，现实里通常拿不到。
x_true = 0.05
v_true = 0.0

# 滤波器估计状态：这是 Kalman Filter 真正输出给外部使用的量。
x_hat = np.array([[0.0], [0.0]], dtype=float)

# P 表示估计误差协方差。
# 初值较大，表示一开始“我对自己的估计不太有把握”。
P = np.eye(2) * 0.1


steps = int(T / dt)
for i in range(steps):
    # ---------- 真实系统更新 ----------
    # 用一个很简单的反馈型加速度命令，让系统动起来。
    # 重点不是控制效果，而是给滤波器提供“有动态变化可估计”的对象。
    a_cmd = -0.8 * x_true - 0.3 * v_true

    # 真实系统实际受到的加速度 = 命令 + 恒定扰动 + 随机过程噪声。
    process_noise = rng.normal(0.0, process_sigma)
    a_true = a_cmd + disturbance + process_noise
    v_true += a_true * dt
    x_true += v_true * dt

    # 传感器只能测位置，并且测量值带噪声。
    z = x_true + rng.normal(0.0, measurement_sigma)

    # ---------- Kalman Predict ----------
    # 先不看传感器，只按模型预测下一时刻状态。
    x_hat = F @ x_hat + B * a_cmd

    # 预测误差协方差也要同步往前推，并叠加过程噪声 Q。
    P = F @ P @ F.T + Q

    # ---------- Kalman Update ----------
    # y 是“创新 / 残差”：测量值减去预测出来的测量值。
    y = np.array([[z]], dtype=float) - H @ x_hat

    # S 是残差协方差，用来综合描述这次残差有多可信。
    S = H @ P @ H.T + R

    # K 是 Kalman 增益，决定本次更新时该信测量多少、信模型多少。
    K = P @ H.T @ np.linalg.inv(S)

    # 用测量修正状态估计。
    x_hat = x_hat + K @ y

    # 融合测量后，不确定性通常会下降。
    P = (np.eye(2) - K @ H) @ P

    # 打印真值、测量值、估计值三者对比。
    if i % 20 == 0:
        print(
            f"time={i*dt:.2f}s | "
            f"x_true={x_true:.4f} | x_meas={z:.4f} | x_hat={x_hat[0,0]:.4f} | "
            f"v_true={v_true:.4f} | v_hat={x_hat[1,0]:.4f}"
        )
