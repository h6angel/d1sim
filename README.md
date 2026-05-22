# d1sim

D1 系列机器人（含 **d1h** 双轮足）的仿真与运控 ROS 2 工作空间。源码在 `src/ddt_ros2_control/`，提供 Gazebo / Mujoco / Webots 仿真、ONNX 强化学习控制器及真机桥接。

## 简介

- **控制器**：`rl_controller`，基于有限状态机 + ONNX 策略
- **仿真**：`gazebo_bridge`、`mujoco_bridge`、`webots_bridge`
- **默认 Gazebo 场景**：`training_arena.world`（16 m × 12 m 室内赛场：外墙、走廊、侧室、迷宫、柱阵、掉头区；障碍间距预留了点云膨胀余量）
- **简单场景**：`empty_world.world`（3 个 1 m 立方障碍，适合快速验证）
- **障碍点云**：`gazebo_obstacle_cloud` 将 Gazebo 中 `obstacle_*` 模型发布为 `/gazebo_obstacles`

更完整的依赖、目录与真机说明见 [`src/ddt_ros2_control/README.md`](src/ddt_ros2_control/README.md)。

## 构建

```bash
cd ~/labpro/d1sim   # 或你的工作空间根目录
colcon build --symlink-install --packages-up-to rl_controller gazebo_bridge gazebo_obstacle_cloud d1h_description
source install/setup.bash
```

需已安装 ROS 2 Humble、`gazebo_ros`、`ros2_control` 等，详见子目录 README。

## Launch（Gazebo）

**默认：d1h + 复杂地图 `training_arena`**

```bash
ros2 launch rl_controller sim_gazebo.launch.py
```

**简单地图（3 个障碍）**

```bash
ros2 launch rl_controller sim_gazebo.launch.py world:=empty_world.world
```

**指定机器人 / 地图**

```bash
ros2 launch rl_controller sim_gazebo.launch.py robot:=d1h world:=training_arena.world
ros2 launch rl_controller sim_gazebo.launch.py robot:=tita world:=empty_world.world
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `robot` | `d1h` | 描述包名，如 `d1h`、`tita`、`d1` |
| `world` | `training_arena.world` | `gazebo_bridge/worlds/` 下的 world 文件 |
| `ns` | `""` | 节点命名空间 |

机器人于原点 `(0, 0, 0.65)` 生成，朝向 **+X** 进入场地。

## 其他仿真（简要）

```bash
# Mujoco
ros2 launch rl_controller sim_mujoco.launch.py robot:=d1h

# Webots
ros2 launch rl_controller sim_webots.launch.py robot:=d1 terrain:=empty_world
```

## 远程仓库

```bash
git remote -v
# origin  git@github.com:h6angel/d1sim.git
```
