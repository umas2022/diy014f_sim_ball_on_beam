import numpy as np

# =========================
# 1. 系统参数
# =========================
g = 9.81        # 重力加速度
dt = 0.01       # 仿真时间步长（越小越稳定）
T = 10.0        # 总仿真时间（秒）

# =========================
# 2. PID 控制器类
# =========================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        # 积分项
        self.integral += error * dt
        
        # 微分项
        derivative = (error - self.prev_error) / dt
        
        # PID 输出
        output = (self.kp * error +
                  self.ki * self.integral +
                  self.kd * derivative)
        
        self.prev_error = error
        return output

# =========================
# 3. 初始化系统状态
# =========================
x = 0.05        # 初始位置（偏离中心5cm）
v = 0.0         # 初始速度

theta = 0.0     # 板子当前角度
theta_dot = 0.0 # 板子角速度（这里简化不用）

x_target = 0.0  # 目标位置（中心）

# =========================
# 4. 创建两个 PID（级联控制）
# =========================

# 外环：位置 -> 目标角度
pid_position = PID(kp=5.0, ki=0.0, kd=2.0)

# 内环：角度 -> 电机控制
pid_angle = PID(kp=20.0, ki=0.0, kd=1.0)

# =========================
# 5. 主循环（仿真）
# =========================
steps = int(T / dt)

for i in range(steps):
    
    # ===== 外环（位置控制）=====
    x_error = x_target - x
    theta_ref = pid_position.update(x_error, dt)
    
    # ===== 内环（角度控制）=====
    theta_error = theta_ref - theta
    motor_cmd = pid_angle.update(theta_error, dt)
    
    # ===== 简化：电机直接控制角度 =====
    # 实际系统中这里是“电机动力学”
    theta += motor_cmd * dt
    
    # ===== 球的动力学 =====
    # x'' = g * theta
    a = g * theta
    
    v += a * dt
    x += v * dt
    
    # ===== 每0.1秒打印一次 =====
    if i % 10 == 0:
        print(f"time={i*dt:.2f}s | x={x:.4f} | v={v:.4f} | theta={theta:.4f}")