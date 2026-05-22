# d1sim

**d1h** 双轮足机器人的 Gazebo 仿真与运控 ROS 2 工作空间。源码在 `src/ddt_ros2_control/`，提供 ONNX 强化学习控制器、Gazebo 桥接及真机硬件桥接。

## 主要功能

- 强化学习控制器（`rl_controller`）：有限状态机 + ONNX 推理
- Gazebo 仿真桥接（`gazebo_bridge`）与障碍点云（`gazebo_obstacle_cloud`）
- `ros2_control` 硬件桥接（`hardware_bridge` + `tita_robot` 驱动库）
- 键盘控制与遥控（ELRS）交互

可选 [Docker 环境说明](src/ddt_ros2_control/docker/README.md)。

## 目录结构（`src/ddt_ros2_control/`）

| 路径 | 说明 |
|------|------|
| `controller/rl_controller` | 强化学习控制器 |
| `hardware/` | `hardware_bridge` 硬件桥接，`tita_robot` CAN 驱动库 |
| `interaction/` | `keyboard_controller`、`teleop_command` |
| `simulation/gazebo_bridge` | Gazebo 仿真与世界文件 |
| `ros_utils/` | ROS 话题命名等工具 |
| `urdfs/d1h_description` | d1h 机器人 URDF / XACRO |

## 环境与依赖

- Ubuntu 22.04、ROS 2 Humble、Gazebo Classic
- ONNX Runtime（按架构选择 x64 或 aarch64）：

```bash
wget https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-x64-1.10.0.tgz
tar xvf onnxruntime-linux-x64-1.10.0.tgz
sudo cp -a onnxruntime-linux-x64-1.10.0/include/* /usr/include
sudo cp -a onnxruntime-linux-x64-1.10.0/lib/* /usr/lib
```

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros ros-humble-gazebo-ros2-control
```

## 构建

```bash
cd ~/labpro/d1sim
colcon build --symlink-install --packages-up-to rl_controller gazebo_bridge gazebo_obstacle_cloud d1h_description
source install/setup.bash
```

真机硬件桥接：

```bash
colcon build --symlink-install --packages-up-to rl_controller hardware_bridge tita_robot
```

交互模块（可选）：

```bash
colcon build --symlink-install --packages-up-to teleop_command keyboard_controller
```

## Gazebo 仿真

**场景**

- `training_arena.world`（默认）：16 m × 12 m 室内赛场
- `empty_world.world`：3 个 1 m 立方障碍，适合快速验证

障碍点云节点 `gazebo_obstacle_cloud` 将 Gazebo 中 `obstacle_*` 模型发布为 `/gazebo_obstacles`。

**启动**

```bash
ros2 launch rl_controller sim_gazebo.launch.py
ros2 launch rl_controller sim_gazebo.launch.py world:=empty_world.world
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `robot` | `d1h` | 固定为 d1h |
| `world` | `training_arena.world` | `gazebo_bridge/worlds/` 下的 world 文件 |
| `ns` | `""` | 节点命名空间 |

机器人在原点 `(0, 0, 0.65)` 生成，朝向 **+X** 进入场地。

## 硬件运行（d1h）

硬件桥接使用 `hardware/tita_robot/lib/*/libtita_robot.so`（厂商驱动库名称），通过 `d1h_description` + `hardware_bridge` 连接真机。

启动前停止已有运控服务：

```bash
sudo systemctl stop joy_controller.service
sudo systemctl stop rl8_controller.service
sudo systemctl stop rl16_controller.service
```

上电后运控板进入 Direct mode（CAN 指令见 [`start.bash`](src/ddt_ros2_control/start.bash)）：

```bash
bash src/ddt_ros2_control/start.bash
ros2 launch rl_controller hw.launch.py
```

## 交互控制

```bash
ros2 run keyboard_controller keyboard_controller_node
ros2 launch teleop_command teleop_command.launch.py
```

## 控制器配置

- 参数：`controller/rl_controller/config/d1h/controllers.yaml`
- ONNX：`config/d1h/flat.onnx`、`squatdown.onnx`
- 状态机：`include/rl_controller/fsm/*`、`src/fsm/*`

## 常见问题

- **控制器未加载**：查看 `controller_manager` 日志与 `controllers.yaml`
- **模型加载失败**：确认已编译 `d1h_description`
- **编译报错**：可将 `#include "rl_controller/rl_controller_parameters.hpp"` 改为 `#include "rl_controller_parameters.hpp"`

## 远程仓库

```bash
git remote -v
# origin  git@github.com:h6angel/d1sim.git
```

## 许可证

各子包许可证见对应 `package.xml` 中的 `license` 字段。
