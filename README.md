# d1sim

D1 系列机器人（含 **d1h** 双轮足）的 Gazebo 仿真与运控 ROS 2 工作空间。源码在 `src/ddt_ros2_control/`，提供 ONNX 强化学习控制器、Gazebo 桥接及真机硬件桥接。

## 主要功能

- 强化学习控制器（`rl_controller`）：有限状态机 + ONNX 推理
- Gazebo 仿真桥接（`gazebo_bridge`）与障碍点云（`gazebo_obstacle_cloud`）
- `ros2_control` 硬件桥接，连接真实机器人驱动
- 键盘控制与遥控（ELRS）交互
- 多机器人模型：`tita`、`d1`（四轮足）、`d1h`（双轮足）

可选 [Docker 环境说明](src/ddt_ros2_control/docker/README.md)，用于可复现部署。

## 目录结构（`src/ddt_ros2_control/`）


| 路径                         | 说明                                       |
| -------------------------- | ---------------------------------------- |
| `controller/rl_controller` | 强化学习控制器                                  |
| `hardware/`                | `hardware_bridge` 硬件桥接，`tita_robot` 底层驱动 |
| `interaction/`             | `keyboard_controller`、`teleop_command`   |
| `simulation/gazebo_bridge` | Gazebo 仿真与世界文件                           |
| `ros_utils/`               | ROS 话题命名等工具                              |
| `urdfs/`                   | 机器人 URDF / XACRO 描述                      |


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
cd ~/labpro/d1sim   # 工作空间根目录
colcon build --symlink-install --packages-up-to rl_controller gazebo_bridge gazebo_obstacle_cloud d1h_description
source install/setup.bash
```

按需追加包，例如硬件桥接、交互模块：

```bash
colcon build --symlink-install --packages-up-to hardware_bridge
colcon build --symlink-install --packages-up-to teleop_command keyboard_controller
```

## Gazebo 仿真

**场景**

- `training_arena.world`（默认）：16 m × 12 m 室内赛场（外墙、走廊、侧室、迷宫、柱阵、掉头区）
- `empty_world.world`：3 个 1 m 立方障碍，适合快速验证

障碍点云节点 `gazebo_obstacle_cloud` 将 Gazebo 中 `obstacle_`* 模型发布为 `/gazebo_obstacles`。

**启动**

默认 d1h + `training_arena`：

```bash
ros2 launch rl_controller sim_gazebo.launch.py
```

简单地图：

```bash
ros2 launch rl_controller sim_gazebo.launch.py world:=empty_world.world
```

指定机器人 / 地图：

```bash
ros2 launch rl_controller sim_gazebo.launch.py robot:=d1h world:=training_arena.world
ros2 launch rl_controller sim_gazebo.launch.py robot:=tita world:=empty_world.world
```


| 参数      | 默认值                    | 说明                                  |
| ------- | ---------------------- | ----------------------------------- |
| `robot` | `d1h`                  | 描述包名，如 `d1h`、`tita`、`d1`            |
| `world` | `training_arena.world` | `gazebo_bridge/worlds/` 下的 world 文件 |
| `ns`    | `""`                   | 节点命名空间                              |


机器人在原点 `(0, 0, 0.65)` 生成，朝向 **+X** 进入场地。

## 硬件运行

硬件桥接依赖 `hardware/tita_robot/lib/*/libtita_robot.so`，需在目标机器上配置 colcon 环境后编译：

```bash
sudo apt install python3-colcon-common-extensions
colcon build --symlink-install --packages-up-to rl_controller hardware_bridge
```

启动前停止已有运控服务：

```bash
sudo systemctl stop joy_controller.service
sudo systemctl stop rl8_controller.service
sudo systemctl stop rl16_controller.service
```

**TITA**：上电后运控板默认 Ready Mode，需运行 `[start.bash](src/ddt_ros2_control/start.bash)` 进入 Direct mode。

```bash
ros2 launch rl_controller hw.launch.py robot:=d1
```

## 交互控制

```bash
colcon build --symlink-install --packages-up-to teleop_command keyboard_controller
source install/setup.bash
```

键盘：

```bash
ros2 run keyboard_controller keyboard_controller_node
```

遥控（ELRS）：

```bash
ros2 launch teleop_command teleop_command.launch.py
```

## 控制器配置

- 参数：`controller/rl_controller/config/<robot>/controllers.yaml`
- ONNX 示例：`config/tita/stand.onnx`、`config/d1/flat.onnx`、`stairs.onnx`
- 状态机：`include/rl_controller/fsm/*`、`src/fsm/*`

更新策略时修改对应 `controllers.yaml` 与 ONNX 路径。

## 官方远程仓库

[https://github.com/DDTRobot/ddt_ros2_control](https://github.com/DDTRobot/ddt_ros2_control)