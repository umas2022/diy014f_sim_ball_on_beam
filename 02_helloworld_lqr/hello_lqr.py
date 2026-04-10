import numpy as np


# =========================
# 1. 系统参数
# =========================
g = 9.81                 # 重力加速度 (m/s^2)
dt = 0.01                # 仿真步长 (s)
T = 10.0                 # 总仿真时间 (s)

# 电机/舵机一阶惯性与限制
beam_tau = 0.05          # 电机时间常数
motor_limit = np.deg2rad(300)   # 电机角速度命令饱和 (rad/s)

# 杆/球限制
theta_limit = np.deg2rad(15)    # 杆最大角度 (±15°)
damping = 0.25                  # 球运动线性阻尼


# =========================
# 2. LQR 所需模型
#    状态: [x, v, theta, theta_dot]
#    输入: u (电机角速度命令)
# =========================
# 这里把球杆系统写成标准线性状态空间形式：
#   x(k+1) = Ad x(k) + Bd u(k)
# 这样后面才能用 LQR 自动求出最优反馈增益 K。
def build_discrete_model(dt):
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

    # 入门示例直接用前向欧拉离散化，便于把“连续模型 -> 离散模型”看清楚。
    ad = np.eye(4) + a * dt
    bd = b * dt
    return ad, bd


def solve_dare(ad, bd, q, r, max_iter=10000, tol=1e-9):
    # 离散 Riccati 方程的迭代求解。
    # 这里的 p 最终会收敛到代价函数对应的稳定解。
    p = q.copy()
    for _ in range(max_iter):
        bt_p = bd.T @ p
        gain_den = r + bt_p @ bd
        p_next = (ad.T @ p @ ad
                  - ad.T @ p @ bd @ np.linalg.inv(gain_den) @ bt_p @ ad
                  + q)
        if np.max(np.abs(p_next - p)) < tol:
            return p_next
        p = p_next
    raise RuntimeError("DARE iteration did not converge")


def lqr_gain(ad, bd, q, r):
    # 先求 Riccati 解 p，再按离散 LQR 公式得到反馈增益 k。
    p = solve_dare(ad, bd, q, r)
    k = np.linalg.inv(r + bd.T @ p @ bd) @ (bd.T @ p @ ad)
    return k


# =========================
# 3. 初始化系统状态
# =========================
x = 0.05         # 初始位置 (m)，偏右 5 cm
v = 0.0          # 初始速度
theta = 0.0      # 杆角度 (rad)
theta_dot = 0.0  # 杆角速度 (rad/s)

x_target = 0.0   # 目标位置


# =========================
# 4. 构建 LQR 控制器
# =========================
ad, bd = build_discrete_model(dt)

# Q 惩罚状态偏差，R 惩罚控制输出大小。
# 数值越大，说明越“不希望”对应状态偏离，或者越“不希望”控制太猛。
q = np.diag([120.0, 12.0, 260.0, 8.0])
r = np.array([[1.5]])
k = lqr_gain(ad, bd, q, r)


# =========================
# 5. 主仿真循环
# =========================
steps = int(T / dt)

for i in range(steps):
    # 当前完整状态向量。
    state = np.array([[x], [v], [theta], [theta_dot]], dtype=float)

    # 目标状态：这里只希望位置、速度、角度、角速度都回到 0。
    target = np.array([[x_target], [0.0], [0.0], [0.0]], dtype=float)
    error_state = state - target

    # LQR 核心公式：u = -Kx
    # 这里 x 用的是“误差状态”，所以本质是在让所有状态都回到目标值。
    motor_cmd = float(-(k @ error_state)[0, 0])

    # 控制命令限幅，避免超出执行器能力。
    motor_cmd = float(np.clip(motor_cmd, -motor_limit, motor_limit))

    # 电机/舵机的一阶惯性响应。
    theta_dot += (motor_cmd - theta_dot) * (dt / beam_tau)
    theta += theta_dot * dt
    theta = float(np.clip(theta, -theta_limit, theta_limit))

    # 小球动力学。
    a = g * theta - damping * v
    v += a * dt
    x += v * dt

    # 输出关键状态，便于观察收敛速度和控制量大小。
    if i % 10 == 0:
        print(f"time={i*dt:.2f}s | x={x:.4f} | v={v:.4f} | "
              f"theta={theta:.4f} | theta_dot={theta_dot:.4f} | u={motor_cmd:.4f}")
