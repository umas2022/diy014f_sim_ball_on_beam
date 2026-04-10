import numpy as np


# =========================
# 1. 基本参数
# =========================
# 这个示例把两个模块串起来：
# - LQR: 根据状态算控制量
# - Kalman Filter: 根据带噪声测量估计状态
# 因此控制律不再是 u = -K x_true，而是 u = -K x_hat。
g = 9.81
dt = 0.01
T = 8.0
beam_tau = 0.05
motor_limit = np.deg2rad(300)
theta_limit = np.deg2rad(15)
damping = 0.25
measurement_sigma = 0.002


def build_discrete_model():
    # 连续时间线性模型：
    # 状态 [x, v, theta, theta_dot]
    # 输入 u 为电机角速度命令
    a = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, -damping, g, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, -1.0 / beam_tau],
    ], dtype=float)
    b = np.array([
        [0.0],
        [0.0],
        [0.0],
        [1.0 / beam_tau],
    ], dtype=float)

    # 用前向欧拉离散化，得到离散时间状态空间模型。
    ad = np.eye(4) + a * dt
    bd = b * dt
    return ad, bd


def solve_dare(ad, bd, q, r, max_iter=10000, tol=1e-9):
    # 离散 Riccati 方程求解，用于得到 LQR 所需的矩阵 p。
    p = q.copy()
    for _ in range(max_iter):
        p_next = (
            ad.T @ p @ ad
            - ad.T @ p @ bd @ np.linalg.inv(r + bd.T @ p @ bd) @ bd.T @ p @ ad
            + q
        )
        if np.max(np.abs(p_next - p)) < tol:
            return p_next
        p = p_next
    raise RuntimeError("DARE did not converge")


def lqr_gain(ad, bd, q, r):
    # 根据 Riccati 解计算离散 LQR 反馈增益。
    p = solve_dare(ad, bd, q, r)
    return np.linalg.inv(r + bd.T @ p @ bd) @ (bd.T @ p @ ad)


rng = np.random.default_rng(1)
ad, bd = build_discrete_model()

# H 矩阵定义“传感器能测到哪些状态”。
# 这里假设能测到：
# - 小球位置 x
# - 杆角 theta
# 测不到：
# - 小球速度 v
# - 杆角速度 theta_dot
h = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
], dtype=float)

# LQR 权重：控制器关心哪些状态偏差，以及控制输出成本。
q_lqr = np.diag([120.0, 12.0, 260.0, 8.0])
r_lqr = np.array([[1.5]])
k_lqr = lqr_gain(ad, bd, q_lqr, r_lqr)

# Kalman Filter 权重：
# q_kf 描述模型不确定性，r_kf 描述测量噪声大小。
q_kf = np.diag([1e-6, 2e-4, 1e-6, 2e-4])
r_kf = np.diag([measurement_sigma ** 2, measurement_sigma ** 2])

# =========================
# 2. 真值状态与估计状态
# =========================
# x_true: 仿真器内部的真实状态
# x_hat: Kalman Filter 给控制器提供的估计状态
x_true = np.array([[0.05], [0.0], [0.0], [0.0]], dtype=float)
x_hat = np.zeros((4, 1), dtype=float)

# 初始协方差较大，表示初始估计并不自信。
P = np.eye(4) * 0.1

steps = int(T / dt)
for i in range(steps):
    # ---------- LQR 控制 ----------
    # 控制器只看估计状态 x_hat，不直接看真实状态 x_true。
    u = float(-(k_lqr @ x_hat)[0, 0])

    # 控制输入限幅，模拟执行器最大能力。
    u = float(np.clip(u, -motor_limit, motor_limit))

    # ---------- 真实系统更新 ----------
    # 真实系统也按相同简化模型运行，但会受到角度限幅约束。
    x_true = ad @ x_true + bd * u
    x_true[2, 0] = float(np.clip(x_true[2, 0], -theta_limit, theta_limit))

    # 传感器测量值 = H x_true + 噪声
    z = h @ x_true + rng.normal(0.0, measurement_sigma, size=(2, 1))

    # ---------- Kalman Predict ----------
    # 利用上一时刻估计状态和已知控制输入，预测下一时刻状态。
    x_hat = ad @ x_hat + bd * u
    P = ad @ P @ ad.T + q_kf

    # ---------- Kalman Update ----------
    # 用传感器新测量到的 z 修正预测结果。
    y = z - h @ x_hat
    S = h @ P @ h.T + r_kf
    K = P @ h.T @ np.linalg.inv(S)
    x_hat = x_hat + K @ y
    P = (np.eye(4) - K @ h) @ P

    # 打印可直接观测的真值与估计值，检查滤波器与闭环控制效果。
    if i % 20 == 0:
        print(
            f"time={i*dt:.2f}s | x_true={x_true[0,0]:.4f} | x_hat={x_hat[0,0]:.4f} | "
            f"theta_true={x_true[2,0]:.4f} | theta_hat={x_hat[2,0]:.4f} | u={u:.4f}"
        )
