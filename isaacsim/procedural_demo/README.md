# Isaac Sim 小球平衡完整入门

这份文档面向第一次接触本项目 Isaac Sim 部分的读者，目标不是只告诉你“怎么运行”，而是尽量让你在读完之后，能够自己理解、运行、修改，并独立做出一个可用的 Isaac Sim 小球平衡 demo。

当前对应的主脚本是 [ball_balance_demo.py](/home/umas/prj/diy014f_sim_ball_on_beam/isaacsim/procedural_demo/ball_balance_demo.py:1)。

这版实现的定位很明确：

- 先做出一版稳定、可教学、便于调参的小球平衡 demo；
- 暂时不追求机械结构和现有 URDF 完全一致；
- 优先让初学者能看懂“场景怎么搭起来、控制怎么闭环、参数怎么调”。

如果你以前只接触过 PID、Gazebo，或者刚开始学 Isaac Sim，这份文档建议从头顺序读一遍。

## 1. 这个目录里有什么

`isaacsim/` 目录当前最重要的文件有两个：

- [ball_balance_demo.py](/home/umas/prj/diy014f_sim_ball_on_beam/isaacsim/procedural_demo/ball_balance_demo.py:1)：完整的 Isaac Sim 脚本，负责启动仿真、创建场景、运行控制器和输出调试日志。
- [README.md](/home/umas/prj/diy014f_sim_ball_on_beam/isaacsim/procedural_demo/README.md:1)：也就是你现在看到的这份文档。

这版没有直接导入现有机械资产，而是纯脚本生成：

- 支撑柱
- 横梁
- 两侧挡边
- 两端止挡
- 小球

这样做的好处是：

- 结构足够简单，便于学习；
- 场景和控制代码在一个文件里，阅读门槛低；
- 每改一个参数，都能很快看到效果。

## 2. 最终效果是什么

这个 demo 想完成的事情很简单：

- 在 Isaac Sim 中生成一根可倾斜的横梁；
- 在横梁上放一颗会滚动的小球；
- 控制横梁的倾角，让小球尽量稳定在目标位置附近。

默认目标位置是横梁中心，也就是：

- `ball_pos_ref = 0.0`

如果运行正常，你会看到：

- 小球初始在中心附近某个偏移位置；
- 横梁开始自动微调；
- 小球逐步回到中心附近；
- 最终横梁只做小幅修正，小球在中心附近保持平衡。

## 3. 先建立直觉：为什么这个系统难控

球杆系统难控，不是因为代码复杂，而是因为“你能控制的量”和“你想稳定的量”不是同一个东西。

你能直接控制的是：

- 横梁角度 `theta`

你真正想稳定的是：

- 小球位置 `x`

但小球位置并不是直接被你命令的。你真正做的是：

1. 先让横梁倾斜一点；
2. 横梁倾斜之后，小球才产生沿梁方向的加速度；
3. 加速度积成速度；
4. 速度再积成位置。

所以这个系统天然容易出现：

- 反应慢半拍；
- 冲过头；
- 左右来回摆；
- 控制器一激进就发散。

这也是为什么这版代码不是“一个简单 PID”就结束，而是加了：

- 状态估计
- 状态反馈
- 限幅
- 执行器平滑
- 小范围保持模式
- 适量物理阻尼

## 4. 这版 demo 的整体结构

建议先把这版系统理解成四层。

### 第 1 层：物理场景

由脚本直接创建：

- 地面
- 支撑柱
- 横梁本体
- 左右挡边
- 两端止挡
- 动态刚体球

横梁长度大约是 `0.26 m`，球半径是 `0.012 m`，球质量是 `0.11 kg`。

### 第 2 层：状态读取

每个物理步都会读取：

- 小球相对横梁的位置
- 小球相对横梁的速度
- 横梁当前角度
- 横梁当前角速度

注意这里不是直接用世界坐标，而是先把小球的世界位置、世界速度转换到横梁坐标系里，这样控制器看到的永远是“沿梁方向”的位置和速度。

### 第 3 层：控制器

控制器在 [BalanceController](/home/umas/prj/diy014f_sim_ball_on_beam/isaacsim/procedural_demo/ball_balance_demo.py:101) 里，核心包括三部分：

- 常速度模型的滤波器 `ConstantVelocityKalmanFilter`
- 主控制模式：LQI 风格状态反馈
- 保持模式：`hold mode`

可以粗略理解成：

- 偏得比较远时，用主控制把球拉回中心；
- 已经接近中心时，切到更温和的保持模式，减少来回抖动。

### 第 4 层：执行器

横梁不是“目标角给多少就立刻到多少”，而是经过 [BeamActuator](/home/umas/prj/diy014f_sim_ball_on_beam/isaacsim/procedural_demo/ball_balance_demo.py:207) 这一层再运动。

这一层做了：

- 目标角低通
- 角速度限幅
- 角加速度限幅
- 接近中心时的压平处理

这是整套系统稳定的关键之一，因为真实执行器也不会瞬间跳到目标姿态。

## 5. 为什么这版比较稳定

这版之所以比最早的 demo 稳，主要是因为它不再试图“看到误差就立刻狠狠干预”。

这里用了几层很重要的稳定化设计：

- 小球位置和速度先做估计，不直接拿噪声较大的瞬时值上控制。
- 控制器里有死区，误差很小时不再频繁修正。
- 有积分项，但积分被限幅，并且在小误差区域会逐步衰减。
- 横梁角度和角速度使用的是阻尼反馈，不再把横梁自身运动继续放大。
- 接近目标时会进入 `hold mode`，动作明显更软。
- 横梁执行器有速度、加速度和平滑约束。
- 小球有显式线阻尼、角阻尼和较高摩擦，避免长期低阻尼游走。

如果把这版控制器简单总结成一句话，那就是：

- 远离中心时积极拉回，接近中心时尽量少打扰。

## 6. 运行前准备

你至少需要满足这几个条件：

- 已经有一个能正常使用的 `isaac` conda 环境；
- 这个环境里已经安装了 Isaac Sim；
- 系统显卡驱动正常；
- Isaac Sim 可以拿到可用 GPU。

### 6.1 激活环境

```bash
conda activate isaac
```

### 6.2 最简单的运行方式

```bash
python isaacsim/procedural_demo/ball_balance_demo.py
```

这会直接打开图形界面，适合你观察实际效果。

### 6.3 无界面快速检查

```bash
python isaacsim/procedural_demo/ball_balance_demo.py --headless --max-steps 1200
```

这个模式适合：

- 快速验证代码有没有跑通；
- 观察日志输出；
- 自动化调参。

## 7. 第一次运行时你应该看到什么

第一次正常运行时，建议你重点观察三件事：

### 1. 场景是否正常生成

你应该能看到：

- 一根横梁
- 两侧挡边
- 一颗橙色小球
- 一个支撑柱

### 2. 小球是否会回中心

小球默认不是从正中央开始，而是有一个小初始偏移：

- `spawn_offset = 0.018`

所以正常现象应该是：

- 小球一开始偏向一侧；
- 横梁开始自动调整；
- 小球向中心回滚；
- 最后在中心附近稳定。

### 3. 终端是否有调试输出

脚本会周期性输出类似这样的日志：

```text
t=4.39 ball_pos=0.0103 ball_vel=0.0050 desired=0.0110 cmd=0.0099 beam=0.0011 beam_rate=0.0130
```

这些字段的意义分别是：

- `t`：仿真时间
- `ball_pos`：小球沿横梁方向的位置
- `ball_vel`：小球沿横梁方向的速度
- `desired`：控制器原始期望横梁角
- `cmd`：经过平滑后的最终命令
- `beam`：横梁当前角度
- `beam_rate`：横梁当前角速度

这个日志非常重要，因为后面大部分调参，都是靠这些量来判断问题属于哪一类。

## 8. 代码是怎么一步一步工作的

如果你想自己写一版类似的系统，推荐按下面顺序理解。

### 8.1 创建 Isaac Sim 应用

脚本先启动：

- `SimulationApp`
- `World`

并设置：

- `physics_dt = 1 / 120`
- `rendering_dt = 1 / 60`

这意味着物理仿真频率是 120 Hz。

对这种控制 demo 来说，120 Hz 是一个比较合适的起点：

- 不算太低，控制不至于太粗；
- 也不至于太高，调试速度还能接受。

### 8.2 创建场景

场景主体由 `FixedCuboid` 和 `DynamicSphere` 直接构成：

- 横梁和挡边是固定几何体；
- 小球是动态刚体球；
- 横梁整体姿态由脚本动态更新。

### 8.3 给小球设置物理属性

当前稳定版给小球加了：

- 较高摩擦
- 线阻尼 `ball_linear_damping`
- 角阻尼 `ball_angular_damping`

这不是“作弊”，而是一个教学 demo 常见的工程化处理。

原因很简单：

- 纯低阻尼球杆系统很容易一直慢慢游走；
- 新手更需要一个先稳定、再理解的系统；
- 等你真的吃透之后，再慢慢把阻尼减小，去追求更真实的动态。

### 8.4 读取状态

每个物理步里，脚本会：

1. 读取小球世界位置和速度；
2. 读取横梁当前角度和角速度；
3. 把小球状态转换到横梁局部坐标系；
4. 把局部位置和局部速度送进控制器。

### 8.5 控制器算目标角

控制器的核心输入是四个量：

- `ball_pos`
- `ball_vel`
- `track_angle`
- `track_rate`

控制器输出两个量：

- `desired_track_angle`
- `final_command`

其中：

- `desired_track_angle` 是原始控制目标；
- `final_command` 是经过平滑后的实际指令。

### 8.6 执行器驱动横梁

`BeamActuator` 不会直接把横梁跳到目标角度，而是按受限速度和加速度慢慢逼近。

所以最终的控制链路是：

1. 读取球的位置和速度
2. 控制器计算目标横梁角
3. 执行器把目标横梁角变成平滑运动
4. 横梁姿态更新
5. 小球在物理世界里重新滚动

这就是完整闭环。

## 9. 控制器详细解释

这一节是最重要的。如果你只想“自己实现一版 Isaac Sim 小球平衡”，这一节必须吃透。

### 9.1 状态估计器

类名是：

- `ConstantVelocityKalmanFilter`

它假设小球在很短时间内近似满足“匀速度模型”，状态为：

- 位置
- 速度

为什么要先滤波？

- 仿真里的速度读数经常比较抖；
- 直接拿瞬时速度上控制，很容易造成命令抖动；
- 滤波后控制器更容易稳定。

你可以把它先简单理解成：

- 一个比普通低通更聪明一点的“位置速度平滑器”。

### 9.2 主控制模式

主控制模式在“球离中心还比较远”时工作。

它的控制目标是：

- 尽快把球拉回中心；
- 同时抑制横梁自身的摆动。

它主要依赖这些增益：

- `lqi_pos_gain`
- `lqi_vel_gain`
- `lqi_integral_gain`
- `lqi_angle_gain`
- `lqi_rate_gain`

可以这样理解：

- `lqi_pos_gain`：看位置偏差有多大，要不要赶紧出手
- `lqi_vel_gain`：看球是不是正在往错误方向冲，提前刹车
- `lqi_integral_gain`：处理长期小偏差
- `lqi_angle_gain`：横梁已经偏了多少，要不要拉回来
- `lqi_rate_gain`：横梁转得太快时，给阻尼

### 9.3 保持模式 `hold mode`

这是这版能明显变稳的重要原因。

进入条件大致是：

- 球已经比较接近中心
- 球速度也比较小

退出条件则设置得更宽一点，这叫“滞回”，目的是避免在边界来回切换。

进入 `hold mode` 后，会发生这些事：

- 目标角限幅更小；
- 位置和速度反馈更温和；
- 横梁角度和角速度阻尼更强；
- 积分项进一步衰减。

它的作用相当于：

- 不是再大力把球往回拽；
- 而是尽量轻柔地“把系统稳稳托住”。

### 9.4 收敛区抑振逻辑

代码里还有一组 `settling_*` 参数。

当系统已经满足：

- 球位置很小
- 球速度很小
- 横梁角度很小
- 横梁角速度很小

控制器会把输出进一步压小。

这一步的目的很明确：

- 避免系统接近收敛时，因为还在不断小修正而重新激起振荡。

## 10. 执行器详细解释

执行器参数主要在 `BeamActuatorConfig` 里。

### `target_angle_alpha`

目标角低通系数。

含义：

- 越小，目标角变化越慢，动作越柔和；
- 越大，横梁越“听话”，但更容易抖。

### `velocity_alpha`

目标角速度的平滑系数。

含义：

- 越小，角速度变化越慢；
- 越大，横梁反应越利落，但也更容易激进。

### `max_velocity`

横梁最大角速度。

这个参数很关键，因为它决定横梁“追目标”的最高速度。

### `max_acceleration`

横梁最大角加速度。

它决定横梁动作是不是会突然冲一下。

### `centering_gain` 和 `centering_deadband`

当横梁目标角已经非常接近 0 时，再额外压一下，让横梁更愿意停在水平附近。

## 11. 当前默认参数的含义

下面这些是当前这版稳定 demo 最重要的默认参数。

### 场景和运行参数

- `physics_dt = 1 / 120`
- `rendering_dt = 1 / 60`
- `beam_angle_limit = 0.08`
- `spawn_offset = 0.018`

### 小球物理参数

- `ball_linear_damping = 0.18`
- `ball_angular_damping = 1.20`
- `ball_material.dynamic_friction = 0.95`
- `ball_material.static_friction = 1.10`

### 主控制参数

- `lqi_pos_gain = 1.45`
- `lqi_vel_gain = 1.20`
- `lqi_integral_gain = 0.20`
- `lqi_angle_gain = 0.78`
- `lqi_rate_gain = 0.30`
- `max_track_angle = 0.022`

### 执行器参数

- `actuator_max_velocity = 0.18`
- `actuator_max_acceleration = 0.9`

### 保持模式相关参数

- `hold_position_entry = 0.005`
- `hold_velocity_entry = 0.018`
- `hold_position_exit = 0.010`
- `hold_velocity_exit = 0.035`
- `hold_max_track_angle = 0.010`

这些值不是“理论最优”，而是“这版 demo 当前稳定、可教学、容易理解”的一组工程参数。

## 12. 最常用的运行命令

### 正常图形运行

```bash
conda activate isaac
python isaacsim/procedural_demo/ball_balance_demo.py
```

### 无界面运行

```bash
conda activate isaac
python isaacsim/procedural_demo/ball_balance_demo.py --headless --max-steps 1200
```

### 调试日志更密一点

```bash
conda activate isaac
python isaacsim/procedural_demo/ball_balance_demo.py --debug-interval 0.25
```

### 改变目标位置

```bash
python isaacsim/procedural_demo/ball_balance_demo.py --ball-pos-ref 0.02
```

### 增大初始扰动，测试恢复能力

```bash
python isaacsim/procedural_demo/ball_balance_demo.py --spawn-offset 0.03
```

### 用更保守的执行器测试稳定性

```bash
python isaacsim/procedural_demo/ball_balance_demo.py --actuator-max-velocity 0.14 --actuator-max-acceleration 0.7
```

## 13. 如何调参

如果你是初学者，最容易踩坑的地方就是“同时改太多参数”。建议每次只改一类，按下面顺序来。

### 第 1 步：先确认是不是物理层问题

先不要急着调控制器，先看这几个现象：

- 小球是不是特别滑，很久都停不下来
- 横梁是不是动作正常，但球就是总在慢慢游走
- 球是不是一撞就弹得很夸张

如果是这些问题，优先看：

- `--ball-linear-damping`
- `--ball-angular-damping`

必要时再去改材质摩擦。

经验上：

- 阻尼太小：系统容易长期小幅游走
- 阻尼太大：球会显得“发粘”，动态不自然

### 第 2 步：再调主控制器

优先调这三个：

- `--lqi-pos-gain`
- `--lqi-vel-gain`
- `--lqi-integral-gain`

它们大致的作用如下。

#### `lqi_pos_gain`

位置误差纠偏强度。

- 太小：球回中心很慢
- 太大：更容易来回摆，甚至冲过头

#### `lqi_vel_gain`

速度阻尼。

- 太小：球容易冲过头
- 太大：系统更稳，但可能反应偏钝

#### `lqi_integral_gain`

消除长期静差。

- 太小：可能会长期停在离中心一点点的位置
- 太大：容易慢慢积累后把系统推成低频振荡

对初学者最稳妥的原则是：

- 先把 `lqi_integral_gain` 设得偏小；
- 用 `lqi_pos_gain + lqi_vel_gain` 先把主体动态调顺；
- 最后再补积分项。

### 第 3 步：再调横梁阻尼项

如果你看到的是“横梁自己摇得比较厉害”，优先看：

- `--lqi-angle-gain`
- `--lqi-rate-gain`

这两个是对横梁自身状态的阻尼反馈。

- `lqi-angle-gain` 更像把横梁往水平位置拉
- `lqi-rate-gain` 更像给横梁转动加刹车

如果横梁动作太疯，通常先加一点 `lqi-rate-gain` 更安全。

### 第 4 步：最后再调执行器

如果控制器算出来的目标没问题，但横梁实际动作太突然，就去调：

- `--actuator-max-velocity`
- `--actuator-max-acceleration`

经验规律：

- 降低 `max_velocity`：通常最能抑制持续摇摆
- 降低 `max_acceleration`：能减少“突然抽一下”

但这两个也不能过低，否则会变成：

- 控制器已经想好了
- 横梁却来不及执行
- 小球又已经滚过去了

### 第 5 步：接近收敛但还微抖时，考虑 hold 模式和阻尼

如果已经基本能平衡，但总在中心附近反复小摆：

- 不要第一反应继续猛加 `lqi_pos_gain`
- 优先考虑提高阻尼或减弱动作

这时优先方向通常是：

- 稍微增大 `ball_linear_damping`
- 稍微增大 `ball_angular_damping`
- 稍微降低 `actuator-max-velocity`
- 适当降低 `lqi_integral_gain`

## 14. 推荐的调参顺序

如果你要从头自己调一套参数，建议按这个顺序：

1. 固定场景尺寸和 `physics_dt`
2. 先把球的物理阻尼调到“不至于长期游走”
3. 用较小 `lqi_pos_gain` 和较大 `lqi_vel_gain` 起步
4. 让球能从 `spawn_offset=0.02` 回中心
5. 再慢慢增大 `lqi_pos_gain` 提高响应速度
6. 最后再补 `lqi_integral_gain`
7. 如果中心附近微抖，优先减激进度，不要先加激进度

一句话总结就是：

- 先求稳，再求快。

## 15. 你可以用哪些现象来判断问题

这一节非常实用。看到不同症状时，应该优先怀疑不同参数。

### 现象 1：小球直接滚到一端

优先检查：

- 控制方向是不是反了
- `lqi_pos_gain` 是否过大
- `max_track_angle` 是否过大
- `actuator-max-velocity` 是否过大

如果代码结构没写错，最常见的原因是：

- 输出太激进，横梁给角太大，球被一路送到端点。

### 现象 2：横梁一直来回快速摆

优先检查：

- `lqi-rate-gain` 是否太小
- `actuator-max-velocity` 是否太大
- `actuator-max-acceleration` 是否太大
- `lqi-angle-gain` 是否太小

### 现象 3：球能回中心，但中心附近一直慢慢游走

优先检查：

- `ball-linear-damping` 是否太小
- `ball-angular-damping` 是否太小
- `lqi-integral-gain` 是否太大
- `actuator-max-velocity` 是否仍然偏快

### 现象 4：球几乎不动，系统太迟钝

优先检查：

- `lqi-pos-gain` 是否太小
- `actuator-max-velocity` 是否太小
- `ball-linear-damping` 是否太大
- `ball-angular-damping` 是否太大

### 现象 5：日志里 `desired` 经常打满限幅

这通常说明主控制器太激进，或者当前物理层太难控。

优先尝试：

- 降低 `lqi-pos-gain`
- 降低 `lqi-integral-gain`
- 增大 `lqi-vel-gain`
- 增大物理阻尼

### 现象 6：日志里 `desired` 不大，但 `beam` 动作还是很猛

优先检查执行器：

- `actuator-max-velocity`
- `actuator-max-acceleration`

## 16. 一套适合新手的实验流程

如果你想用这份代码真正练手，而不是只运行一次，推荐按下面顺序做实验。

### 实验 1：观察基线效果

```bash
python isaacsim/procedural_demo/ball_balance_demo.py
```

目标：

- 先对“正常状态长什么样”建立感觉。

### 实验 2：增加初始偏移

```bash
python isaacsim/procedural_demo/ball_balance_demo.py --spawn-offset 0.03
```

目标：

- 看系统的恢复能力；
- 观察横梁是否会明显更激进。

### 实验 3：让目标点偏到右边

```bash
python isaacsim/procedural_demo/ball_balance_demo.py --ball-pos-ref 0.02
```

目标：

- 理解“平衡”不一定是几何中心，也可以是任意目标位置。

### 实验 4：故意让控制器更激进

```bash
python isaacsim/procedural_demo/ball_balance_demo.py --lqi-pos-gain 1.8 --actuator-max-velocity 0.24
```

目标：

- 观察什么叫“更快但更容易抖”。

### 实验 5：故意让系统更保守

```bash
python isaacsim/procedural_demo/ball_balance_demo.py --lqi-pos-gain 1.2 --lqi-vel-gain 1.3 --actuator-max-velocity 0.14
```

目标：

- 观察什么叫“更稳但更慢”。

只要你做完这五组实验，基本就已经真正理解了这版 demo 的调参逻辑。

## 17. 常见意外情况与处理

这部分建议收藏，实际最常用。

### 17.1 启动时报 GPU 相关错误

如果你看到类似这些报错：

- `no CUDA-capable device is detected`
- `NVIDIA driver is not loaded`
- `Failed to create any GPU devices`

优先检查：

- 当前终端是不是在真实桌面会话下
- 显卡驱动是否正常
- Isaac Sim 是否能访问 GPU
- 是否在某些受限容器/沙箱环境里运行

最简单的系统层检查通常是：

```bash
nvidia-smi
```

如果这里都不正常，先别怪脚本。

### 17.2 能启动 Isaac Sim，但画面不显示

优先检查：

- 是否使用了 `--headless`
- 图形环境是否正常
- 远程桌面/容器是否支持图形显示

如果你只是想验证逻辑，先用：

```bash
python isaacsim/procedural_demo/ball_balance_demo.py --headless --max-steps 1200
```

### 17.3 球掉到横梁外面

如果球直接掉出可控区域，脚本会自动重置。

这通常说明：

- 初始偏移太大
- 控制器太激进
- 物理参数不合适

优先尝试：

- 减小 `--spawn-offset`
- 降低 `--lqi-pos-gain`
- 降低 `--actuator-max-velocity`

### 17.4 明明运行了，但终端没有调试输出

优先检查：

- `--debug-interval` 是否太大
- 是否过早结束了 `--max-steps`

例如：

```bash
python isaacsim/procedural_demo/ball_balance_demo.py --headless --max-steps 1200 --debug-interval 0.25
```

### 17.5 改了参数，但效果变化不明显

通常有三种可能：

- 你改的参数刚好不是当前主导矛盾
- 同时还有别的限幅在起作用
- 改变量太小

例如：

- 你以为在调主控制器
- 但实际上命令一直被 `max_track_angle` 限住

所以调参时不要只看画面，还要看日志里的：

- `desired`
- `cmd`
- `beam`

## 18. 如果你想自己从零实现一版

如果你的目标不是只会跑这份脚本，而是“自己能写出来”，建议按下面这个最小开发顺序来：

1. 在 Isaac Sim 中只生成地面、横梁和球
2. 先不写控制器，只让横梁固定一个角度，观察球滚动
3. 再写一个最简单的位置反馈，让球能大致往回走
4. 加入横梁角度、角速度反馈
5. 加入目标角限幅
6. 加入执行器速度和加速度限制
7. 加入状态估计
8. 最后再加保持模式和更细的抑振逻辑

不要一上来就把所有高级部分都写进去。

真正好用的工程代码，往往也是这样一步步长出来的。

## 19. 这版和 ROS 2 版本是什么关系

这版 Isaac Sim demo 并不是简单照搬 ROS 2 控制器，但两者思路是一脉相承的。

可以对照阅读：

- [balance_controller.py](/home/umas/prj/diy014f_sim_ball_on_beam/ros2/a01_balance_control/a01_balance_control/balance_controller.py:1)

共同点：

- 都在用球位置、球速度、横梁角和横梁角速度做闭环；
- 都有目标角限幅和安全停止思想；
- 都以“先让球稳住”为第一目标。

不同点：

- Isaac Sim 版为了更适合教学和快速试验，加入了更明显的状态估计、保持模式和物理阻尼处理；
- 结构更集中，适合读代码；
- 资产链路更轻，不依赖现有模型导入。

## 20. 当前边界

虽然这版已经足够适合教学和入门，但它依然有意保留了一些简化。

### 它不是高保真机械版本

横梁目前是脚本直接运动学控制，不是真实关节电机链。

### 它更偏“控制教学版”

物理阻尼和摩擦经过了工程化调整，目标是先稳定、好理解、好调参。

### 它不是最终资产工作流

这版没有直接导入现有 `URDF / mesh / USD` 资产。

但也正因为这些简化，它反而非常适合作为 Isaac Sim 控制入门的第一站。

## 21. 一句话总结

如果你只记住一句话，请记这句：

- 小球平衡不是“把增益调大”，而是“让物理、控制器和执行器三层一起配合到刚刚好”。

而这份 [ball_balance_demo.py](/home/umas/prj/diy014f_sim_ball_on_beam/isaacsim/procedural_demo/ball_balance_demo.py:1)，就是一份适合你拆开学习、反复试验、逐步自己实现的 Isaac Sim 小球平衡基线版本。
