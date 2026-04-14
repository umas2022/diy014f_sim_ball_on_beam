# Disturbance Observer Hello World

Disturbance Observer, 简称 DOB，可以把它理解成一种更直接的工程思路：

- 先有一个基础控制器
- 再单独做一个扰动估计器
- 把估计到的扰动从控制量里补偿掉

它和 ADRC 很像，但结构上通常更“模块化”。

## 1. 这个脚本在做什么

脚本 `hello_disturbance_observer.py` 用一个简单的二阶位置系统演示：

- 基础控制器：PD
- 真实系统：除了控制输入外，还带阻尼和外扰
- DOB：估计“真实加速度”和“名义加速度”的差

## 2. 为什么它有用

真实平台里经常会出现：

- 电机左右不对称
- 摩擦变化
- 平台有一点点偏斜
- 球滚动阻力不一致

这些都可以看成扰动或模型失配。  
DOB 的价值就在于：不必把所有这些效应都提前建得很准。

## 3. 脚本逻辑

先用名义模型设计基础控制：

- `theta_pd = (-kp * x - kd * v) / g`

然后加上扰动补偿：

- `theta_cmd = theta_pd - d_hat / g`

其中 `d_hat` 来自扰动观测器。

## 4. `d_hat` 是怎么来的

脚本里做的是最小版 DOB：

- 先算名义加速度 `nominal_a = g * theta_cmd`
- 再看真实加速度和名义加速度差多少
- 用一阶滤波方式把这个差估成 `d_hat`

所以它的本质是：

- 模型说系统应该这么动
- 实际却不是这么动
- 那多出来的部分就当成扰动

## 5. 运行时重点看什么

```bash
python 07_helloworld_disturbance_observer/hello_disturbance_observer.py
```

重点看：

- `x` 能否回零
- `d_hat` 会不会逼近外扰量级

## 6. 和 ADRC 的差别

简单说：

- ADRC 更强调“总扰动 + ESO + 一整套主动补偿”
- DOB 更强调“已有控制器 + 一个附加扰动估计模块”

对工程实践来说，两条路线都非常值得学。
