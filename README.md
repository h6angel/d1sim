# d1sim

D1 机器人仿真与运控 ROS 2 工作空间（`ddt_ros2_ws`）。

本仓库为单一 Git 项目，源码位于 `src/ddt_ros2_control/`，基于 [DDTRobot/ddt_ros2_control](https://github.com/DDTRobot/ddt_ros2_control)。

## 远程仓库

```bash
git remote -v
# origin  git@github.com:h6angel/d1sim.git
```

## 构建

```bash
cd ~/ddt_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

详细功能、仿真与真机启动说明见 [`src/ddt_ros2_control/README.md`](src/ddt_ros2_control/README.md)。
