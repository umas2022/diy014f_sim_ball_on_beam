# Isaac Sim 目录说明

`isaacsim/` 现在拆成两个互相独立的版本，方便分别学习和维护。

## 目录结构

- [procedural_demo/](/home/umas/prj/diy014f_sim_ball_on_beam/isaacsim/procedural_demo)
  - 纯脚本生成几何体的版本，适合初学者先理解 Isaac Sim 中的小球平衡基本原理。
  - 入口脚本：[ball_balance_demo.py](/home/umas/prj/diy014f_sim_ball_on_beam/isaacsim/procedural_demo/ball_balance_demo.py:1)
  - 详细文档：[README.md](/home/umas/prj/diy014f_sim_ball_on_beam/isaacsim/procedural_demo/README.md:1)
- [ros2_model_demo/](/home/umas/prj/diy014f_sim_ball_on_beam/isaacsim/ros2_model_demo)
  - 直接复用 `ros2/a01_balance_1dof` 模型的版本，更贴近你已有的机械结构。
  - 入口脚本：[ball_balance_ros2_model_demo.py](/home/umas/prj/diy014f_sim_ball_on_beam/isaacsim/ros2_model_demo/ball_balance_ros2_model_demo.py:1)
  - 详细文档：[README.md](/home/umas/prj/diy014f_sim_ball_on_beam/isaacsim/ros2_model_demo/README.md:1)

## 先看哪个

如果你是第一次接触 Isaac Sim，建议先从 `procedural_demo/` 开始。

如果你想继续贴近项目自己的 ROS2 模型、关节命名和资产路径，再看 `ros2_model_demo/`。

## 快速运行

程序化版本：

```bash
conda activate isaac
python isaacsim/procedural_demo/ball_balance_demo.py
```

ROS2 模型版本：

```bash
conda activate isaac
python isaacsim/ros2_model_demo/ball_balance_ros2_model_demo.py
```
