# BehaviourTree

# Robocon 2026 R2 行为树决策系统 (BT Logic)

本项目基于 **ROS 2** 与 **BehaviorTree.CPP (v4)** 构建，负责全局任务流转、状态监控与子系统调度。

本系统在仿真中已经经过初步验证，能接入A* Planner完成梅林区导航这一区域的任务，详情可参考算法组群内视频。

##  核心目录结构与说明

```text
bt_logic/
├── bt_xml/
│   └── main_tree.xml            # 行为树核心剧本（分阶段子树设计）
├── include/bt_logic/actions/
│   └── nav2_client.hpp          # Nav2 导航客户端头文件
├── src/
│   ├── bt_main.cpp              # 行为树主循环、节点注册与 Mock 打桩
│   └── nav2_client.cpp          # 异步非阻塞的 ROS 2 Action Client
└── scripts/
    ├── bt_nav_adapter.py        # BT 与 底层寻路算法的胶水适配器（含防死锁护盾）
    └── mock_nav2_server.py      # 用于独立测试的 Fake 导航服务端
CMakelists.txt
package.xml
```
## 编译与运行

### 1. 依赖项
- ROS 2 (Humble / Jazzy)
- [BehaviorTree.CPP v4](https://github.com/BehaviorTree/BehaviorTree.CPP)
- `nav2_msgs`

### 2. 编译
```bash
cd ~/ros2_ws
colcon build --packages-select bt_logic
source install/setup.bash
```

### 3. 单独测试行为树 (Mock 模式)
在底层电控/导航未就绪时，可以使用 Mock Server 进行逻辑空跑：
```bash
# 终端 1: 启动 Mock 导航服务端
python3 src/bt_logic/scripts/mock_nav2_server.py

# 终端 2: 启动行为树主程序
ros2 run bt_logic bt_main
```

##后续开发与优化计划 (TODOs)

- [ ] **消除硬编码坐标**：重构 `nav2_client.cpp`，解析 XML 传入的 `target_pose`（如 `MEILIN_START_POINT`），通过查表映射为真实的 XY 坐标。
- [ ] **引入状态机 (Mission_State)**：在 XML 中加入 Blackboard 变量读取逻辑，废弃“唯位置论”，实现跨区域时的动态子树跳转。
- [ ] **实机接口对齐**：与电控组拉通，将现有的打桩节点（`MockDetectItem` / `MockAlignToPose` 等）替换为真实的 ROS 2 Topic / Action 通信。

---
