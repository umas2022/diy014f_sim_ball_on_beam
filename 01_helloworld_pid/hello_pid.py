import numpy as np


# =========================
# 1. 系统参数
# =========================
# 这个 hello world 使用的是一个“足够简单、但还能说明问题”的球杆线性近似模型。
# 我们控制的是杆子的角速度命令，目标是让小球位置 x 回到 0。
g = 9.81                 # 重力加速度 (m/s^2)
dt = 0.01                # 仿真步长；每隔 dt 更新一次控制与系统状态
T = 10.0                 # 总仿真时间 (s)

# 电机/舵机动态采用一阶惯性近似：
# 角速度不会瞬间等于命令值，而是有一个时间常数 beam_tau。
beam_tau = 0.05
motor_limit = np.deg2rad(300)   # 电机角速度命令饱和上限 (rad/s)

# 杆角和小球运动的简化限制。
theta_limit = np.deg2rad(15)    # 杆最大角度 (rad)
damping = 0.25                  # 小球速度阻尼；用于避免模型无限摆动


# =========================
# 2. PID 控制器类
# =========================
# 这里写一个最小版 PID：
# - kp 对应当前误差
# - ki 对应误差积分
# - kd 对应误差变化率
# 同时加入积分限幅，避免积分项越积越大导致 windup。
class PID:
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral_limit = integral_limit

    def update(self, error, dt):
        # 积分项累加：持续的小误差也会慢慢推动控制输出。
        self.integral += error * dt
        if self.integral_limit is not None:
            # 对积分项做限幅，避免长时间饱和后积分爆掉。
            self.integral = float(np.clip(
                self.integral,
                -self.integral_limit,
                self.integral_limit,
            ))

        # 微分项使用离散差分近似。
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        # PID 三项叠加得到输出。
        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        # 保存本次误差，供下一次计算微分项使用。
        self.prev_error = error
        return output


# =========================
# 3. 系统初始状态
# =========================
# 小球初始在右侧 5 cm，速度为 0，杆一开始是水平静止状态。
x = 0.05         # 小球位置 (m)
v = 0.0          # 小球速度 (m/s)
theta = 0.0      # 杆角度 (rad)
theta_dot = 0.0  # 杆角速度 (rad/s)

x_target = 0.0   # 目标位置：让小球回到中心


# =========================
# 4. 级联 PID
#    外环：位置误差 -> 期望杆角 theta_ref
#    内环：杆角误差 -> 电机角速度命令 motor_cmd
# =========================
# 这就是球杆系统最常见的入门思路：
# - 外环负责“球应该往哪边滚”
# - 内环负责“杆要怎么转到目标角度”
pid_position = PID(kp=9.0, ki=5.0, kd=3.5, integral_limit=1.0)
pid_angle = PID(kp=35.0, ki=3.0, kd=6.0, integral_limit=2.0)


# =========================
# 5. 主仿真循环
# =========================
steps = int(T / dt)

for i in range(steps):
    # ---------- 外环 ----------
    # 位置误差先交给外环 PID，外环不直接输出力，而是输出“目标杆角”。
    x_error = x_target - x
    theta_ref = pid_position.update(x_error, dt)

    # 期望杆角也必须限制在机构允许的范围内。
    theta_ref = float(np.clip(theta_ref, -theta_limit, theta_limit))

    # ---------- 内环 ----------
    # 内环尝试让真实杆角 theta 跟随外环给出的 theta_ref。
    theta_error = theta_ref - theta
    motor_cmd = pid_angle.update(theta_error, dt)

    # 电机角速度命令做饱和，模拟真实执行器限制。
    motor_cmd = float(np.clip(motor_cmd, -motor_limit, motor_limit))

    # ---------- 执行机构 ----------
    # 这里不假设“命令立刻生效”，而是用一阶惯性近似电机/舵机响应。
    theta_dot += (motor_cmd - theta_dot) * (dt / beam_tau)
    theta += theta_dot * dt

    # 杆角本身也有限幅。
    theta = float(np.clip(theta, -theta_limit, theta_limit))

    # ---------- 小球动力学 ----------
    # 简化模型：x'' = g * theta - damping * v
    # 含义是：
    # - 杆向某个方向倾斜会给小球一个沿杆方向的加速度
    # - 阻尼项用于模拟摩擦/耗散，避免理想模型无限振荡
    a = g * theta - damping * v
    v += a * dt
    x += v * dt

    # 每隔一段时间打印一次，便于观察收敛过程。
    if i % 10 == 0:
        print(
            f"time={i*dt:.2f}s | x={x:.4f} | v={v:.4f} | "
            f"theta={theta:.4f} | theta_ref={theta_ref:.4f}"
        )
