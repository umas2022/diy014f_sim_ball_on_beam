# ROS 2 仿真入门

这份文档面向第一次接触本项目 `ROS 2 + Gazebo Sim` 部分的读者，目标是帮助你快速搞清楚目录用途、运行方式、控制链路，以及最常见的调试入口。

## 1. 目录说明

`ros2/` 目录当前包含三个包：

### `a01_balance_1dof`

负责机器人本体描述：

- `urdf/a01_balance_1dof.xacro`: 机构定义；
- `meshes/`: 可视化和碰撞网格；
- `launch/launch.py`: 用 `joint_state_publisher_gui + robot_state_publisher + rviz2` 预览模型。

它更像“机械本体描述包”，适合先确认关节名字、连杆结构和模型显示是否正常。

### `a01_balance_control`

负责控制器节点：

- `a01_balance_control/balance_controller.py`: 控制主逻辑；
- `config/balance_controller.yaml`: 默认参数；
- `launch/control.launch.py`: 单独启动控制器。

控制器会订阅 `/joint_states`，提取：

- `joint_track`: 轨道角度和角速度；
- `joint_ball`: 小球位置和速度。

然后发布轨道关节速度命令到：

- `/model/a01_balance/joint/joint_track/cmd_vel`

### `a01_balance_sim`

负责仿真集成：

- `launch/sim.launch.py`: Gazebo Sim 总启动入口；
- `urdf/a01_balance_sim.xacro`: 用于仿真生成的机器人描述；
- `worlds/empty.sdf`: 默认世界文件。

这个包把 Gazebo、模型生成、桥接和控制器拼成了一个完整闭环。

## 2. 依赖环境

这部分代码默认面向 `ROS 2 Jazzy + Gazebo Sim` 工作流，至少需要这些组件：

- `colcon`
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `ros_gz_sim`
- `ros_gz_bridge`

如果你的 ROS 2 环境已经能正常运行 `ros2`, `colcon`, `rviz2`, `gz sim`，通常就已经具备了基础条件。

## 3. 第一次运行

在仓库根目录执行：

```bash
cd ros2
colcon build
source install/setup.bash
ros2 launch a01_balance_sim sim.launch.py
```

启动后会依次发生这些事情：

1. `gz_sim.launch.py` 打开 Gazebo Sim，并加载 `worlds/empty.sdf`。
2. `robot_state_publisher` 发布 `robot_description`。
3. `ros_gz_bridge parameter_bridge` 建立 ROS 话题和 Gazebo 消息之间的桥接。
4. `a01_balance_control/balance_controller.py` 启动控制器节点。
5. 约 2 秒后，`ros_gz_sim create` 根据 `robot_description` 在 Gazebo 中生成模型。

如果一切正常，你会在 Gazebo 里看到平衡球机构，控制器会尝试把小球稳定在参考位置附近。

## 4. 只查看模型结构

如果你暂时不想启动 Gazebo，只想看 URDF 是否正确，可以运行：

```bash
cd ros2
source install/setup.bash
ros2 launch a01_balance_1dof launch.py
```

这个启动文件会打开：

- `joint_state_publisher_gui`
- `robot_state_publisher`
- `rviz2`

它适合做这些事情：

- 检查关节命名是否一致；
- 看模型朝向是否正确；
- 在进入仿真前先验证 Xacro/URDF 没问题。

## 5. 仿真启动文件做了什么

`a01_balance_sim/launch/sim.launch.py` 是当前最关键的入口文件。它主要包含四部分：

### Gazebo 启动

通过 `ros_gz_sim` 自带的 `gz_sim.launch.py` 启动 Gazebo，并把默认世界设为：

- `a01_balance_sim/worlds/empty.sdf`

### 机器人描述发布

通过 `xacro.process_file(...)` 解析：

- `a01_balance_sim/urdf/a01_balance_sim.xacro`

再把结果交给 `robot_state_publisher` 发布到 `robot_description`。

### 话题桥接

当前桥接了这些核心通道：

- `/clock`
- `/imu`
- Gazebo joint state -> ROS `/joint_states`
- ROS 轨道关节速度命令 -> Gazebo 执行器命令

这一步由 `ros_gz_bridge parameter_bridge` 完成，是 ROS 节点能读到仿真状态、并把控制命令送回 Gazebo 的关键。

### 控制器启动

`a01_balance_control` 会和仿真一起启动，因此 `ros2 launch a01_balance_sim sim.launch.py` 本身就是一个完整的闭环启动命令，不需要你再手动补开控制器。

## 6. 控制器逻辑怎么理解

`balance_controller.py` 不是复杂的最优控制，而是一个很适合入门的双层控制结构：

1. 外环根据小球位置误差和速度，计算“希望轨道达到的倾角”；
2. 内环根据目标倾角与当前倾角差，再结合轨道角速度，计算轨道关节速度命令；
3. 之后再做平滑、步进限幅和速度限幅，减少激烈动作。

可以把它粗略理解成：

- 小球偏左/偏右 -> 想办法让杆子倾斜一点；
- 杆子实际还没到那个角度 -> 再驱动杆子的关节去追这个目标角度；
- 为了仿真更稳，过程中加了滤波和限幅。

### 关键保护机制

控制器里还有一个安全逻辑：

- 当轨道角度超过 `track_angle_limit_for_stop` 时，控制器会直接发 `0` 命令，防止模型姿态发散过大。

## 7. 常用参数

这些参数既定义在 `balance_controller.py`，也在 `config/balance_controller.yaml` 里给了默认值：

- `ball_pos_ref`: 小球目标位置，默认 `0.0`
- `ball_pos_kp`: 位置环比例增益
- `ball_vel_kd`: 位置环速度阻尼
- `track_angle_kp`: 轨道角度环比例增益
- `track_rate_kd`: 轨道角速度阻尼
- `max_track_angle`: 目标轨道角度限幅
- `max_track_velocity`: 最终速度命令限幅
- `command_alpha`: 命令低通滤波系数
- `max_cmd_step`: 单次命令变化量限制
- `ball_pos_alpha`, `ball_vel_alpha`, `track_angle_alpha`, `track_rate_alpha`: 状态滤波系数

你可以直接在启动时覆写参数，例如：

```bash
cd ros2
source install/setup.bash
ros2 launch a01_balance_sim sim.launch.py ball_pos_ref:=0.02 max_track_angle:=0.08
```

这很适合做入门实验，比如：

- 把目标位置改成偏左或偏右，观察小球是否能稳定到新的平衡点；
- 小幅调整 `ball_pos_kp`、`ball_vel_kd`，感受“更快”和“更稳”之间的权衡；
- 调整 `command_alpha` 或 `max_cmd_step`，观察控制命令是否更平滑。

## 8. 调试时最常用的命令

### 查看当前 ROS 话题

```bash
cd ros2
source install/setup.bash
ros2 topic list
```

### 查看关节状态

```bash
cd ros2
source install/setup.bash
ros2 topic echo /joint_states --once
```

### 查看控制命令

```bash
cd ros2
source install/setup.bash
ros2 topic echo /model/a01_balance/joint/joint_track/cmd_vel
```

### 检查控制器日志

控制器会周期性输出类似下面的信息：

- `ball_pos`
- `ball_vel`
- `track`
- `target`
- `cmd`

这些字段非常适合用来判断：

- 小球是否在往目标位置收敛；
- 目标倾角是否过大；
- 命令是否频繁打满限幅。

## 9. 常见问题

### Gazebo 打开了，但看不到模型

优先检查：

- `robot_state_publisher` 是否成功启动；
- `ros_gz_sim create` 是否报错；
- `spawn_z` 是否设置得太低，导致模型与地面重叠。

当前默认 `spawn_z` 是 `0.08`。

### 控制器启动了，但球完全不动

优先检查：

- `/joint_states` 是否真的有数据；
- 关节名字是否和控制器配置一致；
- `track_cmd_topic` 是否和桥接配置一致；
- `ros_gz_bridge` 是否正常工作。

当前默认关节名是：

- `track_joint_name=joint_track`
- `ball_joint_name=joint_ball`

### 一启动就发散

这通常说明增益偏大、方向不对，或者模型坐标定义与控制假设不一致。可以先尝试：

- 减小 `ball_pos_kp`
- 减小 `track_angle_kp`
- 检查是否需要调整 `control_sign`

## 10. 建议的学习顺序

如果你是第一次把控制算法迁移到 ROS 2 仿真，推荐这样看：

1. 先运行 `a01_balance_1dof`，确认模型和关节。
2. 再运行 `a01_balance_sim`，理解 Gazebo、桥接和控制器是怎么串起来的。
3. 阅读 `balance_controller.py`，把每个参数和实际现象对应起来。
4. 最后开始自己改参数或替换控制律。

## 11. 后续可以继续做什么

- 给控制器加入更系统的状态估计；
- 把当前双层 PD 控制器换成 `LQR / LQG / MPC`；
- 增加话题录包、数据可视化和参数对比实验；
- 加入更真实的摩擦、噪声和扰动模型。
