# BehaviourTree

# 🤖 Robocon 2026 R2 行为树决策系统 (BT Logic)

本项目是 Robocon 2026 R2 机器人的“中枢大脑”。基于 **ROS 2** 与 **BehaviorTree.CPP (v4)** 构建，负责全局任务流转、状态监控与子系统调度。

本系统采用**“高阶异步解耦”**架构设计，实现了决策层（BT）与执行层（规划/电控）的彻底分离，支持在 MuJoCo 仿真与物理实机之间实现 **零代码修改** 的无缝切换。

## 📁 核心目录结构与说明

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
```

## 🧠 系统架构数据流向

本系统严格遵循 **10Hz 大脑心跳 + 异步动作执行** 的机制：

1. **决策层 (10Hz)**：`bt_main.cpp` 以 10Hz 频率 Tick XML 树。遇到导航节点时，发送 Action Goal，随后立即将节点挂起（RUNNING），**绝不阻塞主循环**。
2. **契约层 (Adapter)**：`bt_nav_adapter.py` 接管指令，向 A* 规划器打出 `/task/trigger` 触发枪。
3. **监控护盾 (Anti-Deadlock)**：适配器后台循环测距。若检测到底层规划器罢工（空路径），将开启 `1.2s` 滴答缓冲；若超时未恢复，强行中止并向 BT 报错，防止整车植物人死锁。

## 🚀 编译与运行指南

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

### 4. 接入 A* 寻路与底盘仿真联调
```bash
# 终端 1: 启动导航胶水层适配器
python3 src/bt_logic/scripts/bt_nav_adapter.py

# 终端 2: 启动行为树大脑
ros2 run bt_logic bt_main
```

##后续开发与优化计划 (TODOs)

- [ ] **消除硬编码坐标**：重构 `nav2_client.cpp`，解析 XML 传入的 `target_pose`（如 `MEILIN_START_POINT`），通过查表映射为真实的 XY 坐标。
- [ ] **引入状态机 (Mission_State)**：在 XML 中加入 Blackboard 变量读取逻辑，废弃“唯位置论”，实现跨区域时的动态子树跳转。
- [ ] **全局 Mux 路由**：将底盘控制权交由 `cmd_vel_mux` 管理，避免 `MockAlignToPose` 与底层寻路算法抢夺方向盘。
- [ ] **实机接口对齐**：与电控组拉通，将现有的打桩节点（`MockDetectItem` / `MockAlignToPose` 等）替换为真实的 ROS 2 Topic / Action 通信。

---
