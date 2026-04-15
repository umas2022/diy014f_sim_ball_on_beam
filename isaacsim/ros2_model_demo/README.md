# Isaac Sim ROS2 模型版入门

这份文档对应 [ball_balance_ros2_model_demo.py](/home/umas/prj/diy014f_sim_ball_on_beam/isaacsim/ros2_model_demo/ball_balance_ros2_model_demo.py:1)。

如果说 `isaacsim/procedural_demo/` 的目标是“先在 Isaac Sim 里做出一版稳定、容易理解、容易调参的小球平衡 demo”，那么这个目录的目标就是另一件事：

- 不再手工拼一个教学用横梁场景；
- 直接复用 `ros2/a01_balance_1dof` 里的 `xacro/URDF/mesh`；
- 让 Isaac Sim 真正控制你项目里的 `joint_track` 和 `joint_ball`。

现在这版已经能稳定演示，小球也已经能在 Isaac Sim 中保持平衡。它的价值不只是“又做了一版 demo”，而是打通了从 ROS2 机械模型到 Isaac Sim 物理控制的整条链路。

## 1. 这个版本到底做了什么

脚本启动后，核心流程如下：

1. 调用 `xacro` 展开 `ros2/a01_balance_1dof/urdf/a01_balance_1dof.xacro`
2. 把 URDF 里的 `package://...` 资源路径替换成仓库内的绝对路径
3. 通过 Isaac Sim 的 URDF importer 导入模型
4. 获取导入 articulation 中的 `joint_track` 和 `joint_ball`
5. 读取两个 DOF 的位置与速度
6. 用控制器计算目标横梁角
7. 再通过速度环把目标横梁角变成 `joint_track` 的速度目标
8. 当球到达行程边界附近时自动 reset，便于重复测试

这里最关键的一点是：

- `procedural_demo/` 控制的是脚本自己创建出来的简化横梁和球
- `ros2_model_demo/` 控制的是从 ROS2 模型导入后的真实关节

所以这版更接近后续真实项目接入，而不只是一个教学示例。

## 2. 它和 procedural 版本的区别

两版 demo 的控制目标相同，都是让球尽量回到中心；但实现路径明显不同。

### 2.1 场景来源不同

`procedural_demo/`：

- 横梁、挡边、球、支撑柱都由 Python 脚本直接创建
- 几何、质量、阻尼、摩擦都由 demo 作者统一定义
- 整个系统的自由度和接触关系比较“可控”

`ros2_model_demo/`：

- 横梁和球来自 `a01_balance_1dof.xacro`
- 需要经过 `xacro -> URDF -> Isaac importer`
- 导入后的关节、惯量、限位、碰撞体、mesh 层级都由模型本身决定

这意味着模型版从一开始就不是“从零捏一个最容易平衡的场景”，而是在适应已有模型。

### 2.2 状态获取方式不同

`procedural_demo/`：

- 小球是一个独立刚体
- 需要把世界坐标下的位置、速度转换到横梁坐标系
- 控制器看到的是“沿梁方向”的连续物理量

`ros2_model_demo/`：

- 小球位置直接来自 `joint_ball`
- 横梁角直接来自 `joint_track`
- 状态读取走的是 imported articulation 的 DOF 接口

模型版的状态更“工程化”，但前提是导入后的 DOF 命名、绑定和物理行为都必须正确。

### 2.3 执行器路径不同

`procedural_demo/`：

- 横梁姿态最终由脚本自己控制
- 执行器行为和限幅逻辑都在 demo 内部
- 只要本地控制逻辑正确，系统就能工作

`ros2_model_demo/`：

- 控制器先给出目标横梁角
- 再经 `BeamActuator` 平滑成目标角轨迹
- 再经 `VelocityCommandController` 转成 `joint_track` 速度命令
- 最后通过 Isaac Sim `dynamic_control` 写入导入关节

也就是说，模型版不仅多了一层导入模型，还多了一层“目标角 -> 关节速度命令”的接口适配。

### 2.4 项目定位不同

可以简单理解为：

- `procedural_demo/` 更适合讲控制思路、搭场景方法和调参直觉
- `ros2_model_demo/` 更适合继续往真实模型复用、ROS2 对接、后续系统集成推进

如果后面你想做的事情是：

- 替换成你自己的控制器
- 接 ROS2 bridge
- 复用现有机器人描述和关节命名
- 让 Isaac Sim 更像真实项目的预演环境

应该在这个目录上继续迭代。

## 3. 为什么这个版本调试花了更多时间

这是这份文档最值得说清楚的地方。

模型版不是单纯因为“控制器更难”才花更多时间，而是因为它把原来 procedural 版本里被简化掉的工程问题，全部重新引入了。

### 3.1 先要打通模型导入链路

在 procedural 版本里，没有模型导入问题，因为场景完全由脚本生成。

在模型版里，首先要确认这些环节都成立：

- `xacro` 能正常展开
- `package://` 路径能被替换成 Isaac Sim 可访问的真实路径
- Isaac 的 URDF importer 能成功创建 articulation
- 导入后的根 prim 路径、关节层级和预期一致

只要其中任何一环不对，后面的控制代码就没有意义。

### 3.2 关节“存在”不等于关节“可用”

模型被导入成功，并不代表控制已经接上。

真正需要确认的是：

- `dynamic_control` 能否拿到 articulation handle
- `joint_track` 和 `joint_ball` 的 DOF 名称是否和预期一致
- 读取到的位置、速度是否真的是我们想要的物理量
- 写入的速度目标是否真的作用在横梁关节上

这类问题在 procedural 版本里基本不存在，因为对象都是脚本亲手创建和绑定的；在模型版里，它们都是必须逐项验证的。

### 3.3 导入模型的物理特性更接近真实，但也更难伺候

procedural 版本之所以更容易快速稳定，一个重要原因是它的物理结构更“理想化”：

- 几何简单
- 自由度简单
- 接触关系简单
- 参数是为 demo 稳定性服务的

模型版则不同。它更接近真实机器人描述，因此会带来额外敏感性：

- 关节阻尼、驱动模式和限位不能随便假设
- 球和轨道的接触行为更容易暴露细节问题
- 惯量和摩擦不再是“为了好调而设置”的一组理想值
- 一些 procedural 版可用的控制参数，直接搬过来未必稳定

这也是为什么模型版虽然最终也能稳住，但中间会比 procedural 版更容易出现：

- 方向对了但响应过慢
- 方向对了但来回抽动
- 小球快到中心时仍然持续修正
- 球在边界附近突然跑飞

### 3.4 控制链路更长，问题定位更慢

procedural 版里，控制链路大致是：

- 读取球状态
- 算目标角
- 直接控制横梁

模型版里，链路变成：

- 读取 `joint_ball` / `joint_track`
- 估计状态
- 算目标角
- 目标角经 `BeamActuator` 平滑
- 再经速度环变成 DOF 速度命令
- 最终由 imported articulation 执行

链路每多一层，就多一层需要排查的可能性：

- 是控制器方向错了？
- 是目标角太激进？
- 是执行器限速太小？
- 是速度环阻尼不够？
- 还是 imported joint drive 自己在和控制器“打架”？

这也是这版调试花费更久的核心原因之一。很多时候不是“算法不会写”，而是要先判断问题到底出在哪一层。

### 3.5 为了得到稳定演示，必须额外做很多保守处理

现在能稳定演示，不是因为导入后一次就成，而是因为后面又补了几层稳定化处理，例如：

- 给 `joint_track` 单独设 drive 属性，避免 importer 默认行为干扰
- 提高 articulation solver iteration，减小数值不稳定
- 给球增加额外线阻尼和角阻尼
- 对横梁角、角速度、角加速度都做限幅
- 引入 `hold mode`，避免已经接近中心时还大动作纠偏
- 球到达边界附近自动 reset，避免一次失稳后难以继续观察

这些处理在 procedural 版里也有一部分思想，但在模型版里更重要，因为模型版本身的物理响应更敏感。

一句话总结：

- procedural 版是在“理想化 demo 场景”里把控制做稳
- 模型版是在“真实导入模型链路”上把控制、接口和物理行为一起做稳

后者自然更耗时。

## 4. 当前版本为什么值得继续保留

尽管它比 procedural 版更难调，但这版一旦打通，收益也更高：

- 后续替换成别的控制器更自然
- 后续接 ROS2 bridge 更自然
- 后续验证 URDF/xacro 修改对仿真的影响更直接
- 后续做从仿真到实体的一致性分析也更有意义

所以它花更多时间，不是“走弯路”，而是在补齐工程真实性带来的那部分工作。

## 5. 运行方式

图形界面运行：

```bash
conda activate isaac
python isaacsim/ros2_model_demo/ball_balance_ros2_model_demo.py
```

无界面调试：

```bash
python isaacsim/ros2_model_demo/ball_balance_ros2_model_demo.py --headless --max-steps 240 --debug-interval 0.25
```

## 6. 当前默认策略

为了优先得到“默认启动就尽量稳”的效果，这版默认做了比较保守的设置：

- `spawn_offset` 默认是 `0.006 m`
- `control_sign` 默认是 `1`
- 控制器目标角默认限幅为 `0.05 rad`
- 横梁执行器有速度和加速度限幅
- `joint_track` 最终走速度命令，而不是每步强行改关节位置
- 小球接近行程边界会自动 reset

这使它更适合做：

- 稳定演示
- 关节接口验证
- 参数微调

而不是一开始就拿极端初始条件去硬测上限。

## 7. 最值得优先关注的调参项

如果你想继续优化，优先看下面这些参数：

- `--spawn-offset`
  - 初始偏移越大，越考验控制和接触稳定性
- `--control-sign`
  - 方向错了会越控越偏
- `--max-track-angle`
  - 太小来不及救，太大容易过冲
- `--actuator-max-velocity`
  - 决定横梁目标角跟踪速度
- `--track-vel-kp`
  - 决定目标角到关节速度命令的反应强度
- `--track-vel-kd`
  - 抑制横梁关节的速度抖动
- `--track-drive-damping`
  - 导入关节本身的阻尼设置，过大过小都可能影响手感

建议调参顺序：

1. 先固定较小的 `spawn-offset`
2. 确认 `control-sign` 正确
3. 调 `max-track-angle` 和 `actuator-max-velocity`
4. 再调 `track-vel-kp` / `track-vel-kd`
5. 最后再细调 LQI 增益

## 8. 两个版本该怎么选

如果你的目标是：

- 先理解控制逻辑
- 先学习 Isaac Sim 里怎么搭一个球杆系统
- 想快速做实验和调参

优先看 `isaacsim/procedural_demo/`。

如果你的目标是：

- 复用项目现有 ROS2 机械模型
- 验证 `xacro/URDF` 到 Isaac Sim 的接入
- 在真实关节命名和导入 articulation 上继续开发

优先看 `isaacsim/ros2_model_demo/`。

最实际的理解方式是：

- `procedural_demo/` 解决“怎么把问题讲清楚、做稳定”
- `ros2_model_demo/` 解决“怎么把真实模型接进来、也做稳定”
