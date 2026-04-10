import numpy as np


# =========================
# 1. 基本参数
# =========================
# 这里用一个非常简化的“有限时域最优控制”来说明 MPC 核心思想：
# - 不是只看当前一步
# - 而是往前看 horizon 步
# - 每次只执行求出来的第一步控制
g = 9.81
dt = 0.02
T = 6.0
damping = 0.25
theta_limit = np.deg2rad(12)
horizon = 15


# =========================
# 2. 离散模型
#    状态: [x, v]
#    输入: theta_cmd
# =========================
# 这里把杆角命令直接看成控制输入，重点突出 MPC“预测 + 优化”的结构。
F = np.array([
    [1.0, dt],
    [0.0, 1.0 - damping * dt],
], dtype=float)

G = np.array([
    [0.0],
    [g * dt],
], dtype=float)

# Q、R、Qf 定义有限时域代价函数：
# - Q: 中间每一步的状态偏差代价
# - R: 控制动作代价
# - Qf: 终端状态代价
Q = np.diag([80.0, 8.0])
R = np.array([[1.0]])
Qf = np.diag([120.0, 12.0])


def finite_horizon_gain(F, G, Q, R, Qf, horizon):
    # 有限时域 Riccati 反向递推。
    # 从终端代价 Qf 开始往回推，得到每一步对应的反馈增益。
    p = Qf.copy()
    gains = []
    for _ in range(horizon):
        k = np.linalg.inv(R + G.T @ p @ G) @ (G.T @ p @ F)
        gains.append(k)
        p = Q + F.T @ p @ (F - G @ k)

    # 递推是从后往前算的，最后再翻转成正向时间顺序。
    gains.reverse()
    return gains


gains = finite_horizon_gain(F, G, Q, R, Qf, horizon)

# 初始状态：小球偏离中心 5 cm，初速度为 0。
x = np.array([[0.05], [0.0]], dtype=float)
steps = int(T / dt)

for i in range(steps):
    # MPC 的 receding horizon（滚动时域）思想：
    # 虽然内部规划了未来 horizon 步，但当前只执行第 1 步控制。
    theta_cmd = float(-(gains[0] @ x)[0, 0])

    # 杆角命令限幅，模拟真实机构的角度约束。
    theta_cmd = float(np.clip(theta_cmd, -theta_limit, theta_limit))

    # 系统按离散模型前进一步。
    x = F @ x + G * theta_cmd

    # 输出状态与控制量，观察系统是否逐步回中。
    if i % 10 == 0:
        print(
            f"time={i*dt:.2f}s | x={x[0,0]:.4f} | v={x[1,0]:.4f} | theta_cmd={theta_cmd:.4f}"
        )
