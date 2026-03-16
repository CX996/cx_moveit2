# CR7 Robot Controller 设计文档

---

# 1 系统概述

CR7 Robot Controller 是一个面向工业焊接应用的机器人控制系统，基于
**ROS 2** 与 **MoveIt 2** 构建。

系统主要用于实现：

* 机器人运动控制
* 路径规划与避障
* 轨迹执行
* 焊接工艺控制
* 规划场景管理
* 机器人状态管理

系统通过统一控制接口，为上层应用提供完整的机器人控制能力。

---

# 2 设计目标

系统设计遵循以下原则：

### 2.1 模块化设计

各功能模块独立，职责清晰，降低系统耦合度。

---

### 2.2 规划与执行分离

运动规划与轨迹执行由不同模块负责，提高系统稳定性与可扩展性。

---

### 2.3 统一用户接口

通过统一的控制器 API 向用户提供所有功能接口，屏蔽底层复杂性。

---

### 2.4 高可扩展性

支持未来扩展：

* 新规划器
* 视觉系统
* 力控系统
* 多机器人协作

---

# 3 系统总体架构

系统采用 **五层架构设计**：

```
Application Layer
（应用层）

User API Layer
（用户接口层）

Task Management Layer
（任务管理层）

Motion Planning Layer
（运动规划层）

Device Control Layer
（设备控制层）
```

系统结构示意：

```
┌──────────────────────────────┐
│        Application Layer      │
│    焊接任务 / UI / 工艺流程    │
└──────────────────────────────┘
               │
┌──────────────────────────────┐
│        User API Layer         │
│      CR7RobotController       │
└──────────────────────────────┘
               │
┌──────────────────────────────┐
│     Task Management Layer     │
│   任务调度 / 轨迹执行管理     │
└──────────────────────────────┘
               │
┌──────────────────────────────┐
│     Motion Planning Layer     │
│   路径规划 / IK / 轨迹生成    │
└──────────────────────────────┘
               │
┌──────────────────────────────┐
│     Device Control Layer      │
│       机器人SDK接口           │
└──────────────────────────────┘
```

---

# 4 User API Layer（用户接口层）

用户接口层是系统对外提供的统一控制入口。

核心类：

```
CR7RobotController
```

职责：

* 封装所有系统功能
* 提供统一 API
* 管理系统生命周期

---

# 4.1 系统初始化接口

### 系统初始化

```
initializeSystem()
```

功能：

* 初始化 ROS2 节点
* 初始化规划模块
* 初始化机器人 SDK
* 初始化规划场景

---

### 系统关闭

```
shutdownSystem()
```

---

# 4.2 机器人基础控制接口

### 上电 / 下电

```
powerOn()
powerOff()
```

---

### 机器人使能

```
enableRobot()
disableRobot()
```

---

### 机器人复位

```
resetRobot()
```

清除错误状态。

---

# 4.3 初始化运动接口

工业机器人系统必须提供 **初始化运动功能**。

---

### 回零点（Home）

```
moveToHome()
```

说明：

机器人回到 **关节零位姿态**。

用途：

* 系统启动
* 故障恢复
* 校准

---

### 回初始点（Start Position）

```
moveToInitialPose()
```

说明：

机器人回到 **工作起始位置**。

例如：

```
焊接准备位置
```

---

### 运动停止

```
stopMotion()
```

立即停止机器人运动。

---

# 4.4 机器人状态接口

### 获取机器人状态

```
getRobotState()
```

返回：

* 当前关节角
* TCP位姿
* 机器人模式
* 错误状态

---

### 获取关节状态

```
getJointState()
```

---

### 获取TCP位姿

```
getTcpPose()
```

---

# 4.5 运动控制接口

---

### 关节运动

```
moveJoint(joint_target)
```

---

### 直线运动

```
moveLinear(target_pose)
```

---

### 圆弧运动

```
moveCircular(pose1, pose2)
```

---

### Jog控制

```
jogJoint()
jogCartesian()
```

用于手动调试。

---

# 4.6 规划接口

---

### 规划到目标位姿

```
planToPose(target_pose)
```

---

### 规划到关节目标

```
planToJoint(target_joint)
```

---

### 多路点规划

```
planWaypoints(waypoints)
```

---

### 规划并执行

```
planAndExecute(target)
```

---

# 4.7 规划场景管理接口

用于管理规划环境。

---

### 添加障碍物

```
addBoxObstacle()
addCylinderObstacle()
addMeshObstacle()
```

---

### 删除障碍物

```
removeObstacle()
```

---

### 清空场景

```
clearScene()
```

---

### 更新障碍物

```
updateObstaclePose()
```

---

### 场景保存 / 加载

```
saveScene()
loadScene()
```

---

# 4.8 焊接控制接口

---

### 到达焊接起点

```
moveToWeldStart(start_pose)
```

执行过程：

```
当前位姿
   │
安全高度
   │
避障规划
   │
焊接起点
```

---

### 执行焊接轨迹

```
executeWeldTrajectory()
```

---

### 起弧

```
startArc()
```

---

### 停弧

```
stopArc()
```

---

### 摆焊控制

```
enableWeaving()
disableWeaving()
setWeavingParameters()
```

参数包括：

* 摆动幅度
* 摆动频率
* 停留时间

---

# 5 Task Management Layer（任务管理层）

负责 **任务调度与执行管理**。

主要模块：

```
TrajectoryManager
TrajectoryExecutor
Scheduler
SceneManager
```

---

# 5.1 Trajectory Manager

负责轨迹管理：

* 轨迹缓存
* 轨迹队列
* 轨迹暂停
* 轨迹恢复

---

# 5.2 Trajectory Executor

负责轨迹执行：

* 轨迹发送
* 实时跟踪
* 执行监控
* 异常处理

---

# 5.3 Scheduler

任务调度模块：

* FIFO任务队列
* 优先级调度
* 任务中断
* 任务恢复

---

# 5.4 Scene Manager

负责规划场景管理：

* 场景初始化
* 障碍物管理
* 场景更新

---

# 6 Motion Planning Layer（运动规划层）

负责路径规划。

模块：

```
MotionPlanner
IKSolver
TrajectoryGenerator
TrajectoryOptimizer
ConstraintManager
CollisionManager
```

---

# 6.1 Motion Planner

支持规划器：

* OMPL
* Pilz
* Cartesian Planner

功能：

* 单点规划
* 多点规划
* 避障规划

---

# 6.2 IK Solver

功能：

* 逆运动学求解
* 多解筛选
* 奇异点检测
* 碰撞检测

---

# 6.3 Trajectory Generator

生成轨迹：

* 关节轨迹
* 笛卡尔轨迹

插值方法：

* 五次多项式
* 样条插值

---

# 6.4 Trajectory Optimizer

轨迹优化：

* 时间参数化
* 速度限制
* 加速度限制
* jerk限制

---

# 6.5 Constraint Manager

支持约束：

* 位置约束
* 姿态约束
* 关节约束

---

# 6.6 Collision Manager

碰撞检测：

* 机器人自碰撞
* 环境碰撞
* 最小距离计算

---

# 7 Device Control Layer（设备控制层）

负责机器人硬件控制。

模块：

```
RobotSDKInterface
RobotMotionInterface
RobotStateManager
SafetyManager
```

---

# 7.1 Robot SDK Interface

负责：

* SDK初始化
* 连接管理
* 错误处理

---

# 7.2 Robot Motion Interface

基础运动：

```
moveJ
moveL
moveC
servoJ
servoL
speedJ
speedL
```

---

# 7.3 Robot State Manager

管理：

* 当前关节
* TCP位姿
* 运动状态
* 错误状态

---

# 7.4 Safety Manager

安全保护：

* 软限位
* 硬限位
* 碰撞保护
* 急停保护

---

# 8 系统运行流程

系统启动流程：

```
系统启动
   │
初始化系统
   │
机器人上电
   │
机器人使能
   │
回零点
   │
回初始点
   │
进入工作状态
```

---

# 9 错误处理机制

错误分类：

| 类型   | 描述      |
| ---- | ------- |
| 规划错误 | IK失败    |
| 执行错误 | 轨迹执行失败  |
| 通信错误 | SDK通信异常 |
| 系统错误 | 硬件异常    |

错误处理流程：

```
错误检测
  ↓
错误分类
  ↓
错误记录
  ↓
自动恢复 / 上报
```

---

# 10 系统扩展能力

支持扩展：

### 视觉系统

用于焊缝识别。

---

### 力控系统

用于接触控制。

---

### 多机器人系统

支持：

* 双机器人焊接
* 协作机器人

---

# 11 CR7 Robot Controller 功能总表

本表列出了系统提供的全部核心功能接口及其所属模块。

---

# 11.1 系统管理功能

| 功能名称   | 接口                   | 功能说明       |
| ------ | -------------------- | ---------- |
| 系统初始化  | `initializeSystem()` | 初始化系统各模块   |
| 系统关闭   | `shutdownSystem()`   | 关闭系统并释放资源  |
| 系统复位   | `resetRobot()`       | 清除机器人错误状态  |
| 系统状态查询 | `getSystemState()`   | 获取系统整体运行状态 |

---

# 11.2 机器人基础控制

| 功能名称   | 接口                | 功能说明       |
| ------ | ----------------- | ---------- |
| 机器人上电  | `powerOn()`       | 机器人控制柜上电   |
| 机器人下电  | `powerOff()`      | 机器人控制柜断电   |
| 机器人使能  | `enableRobot()`   | 机器人进入可运动状态 |
| 机器人去使能 | `disableRobot()`  | 禁止机器人运动    |
| 急停     | `emergencyStop()` | 立即停止机器人运动  |
| 运动停止   | `stopMotion()`    | 正常停止当前运动   |

---

# 11.3 机器人初始化运动

| 功能名称 | 接口                    | 功能说明       |
| ---- | --------------------- | ---------- |
| 回零点  | `moveToHome()`        | 机器人回到关节零位  |
| 回初始点 | `moveToInitialPose()` | 回到系统默认起始位置 |
| 安全位姿 | `moveToSafePose()`    | 移动到安全位置    |
| 回工作点 | `moveToWorkPose()`    | 回到工作准备位置   |

---

# 11.4 机器人状态查询

| 功能名称    | 接口                 | 功能说明      |
| ------- | ------------------ | --------- |
| 获取机器人状态 | `getRobotState()`  | 获取机器人运行状态 |
| 获取关节状态  | `getJointState()`  | 获取关节角度信息  |
| 获取TCP位姿 | `getTcpPose()`     | 获取工具末端位姿  |
| 获取运动状态  | `getMotionState()` | 查询当前运动状态  |
| 获取错误信息  | `getErrorState()`  | 获取错误码     |

---

# 11.5 机器人运动控制

| 功能名称   | 接口               | 功能说明    |
| ------ | ---------------- | ------- |
| 关节运动   | `moveJoint()`    | 关节空间运动  |
| 直线运动   | `moveLinear()`   | 笛卡尔直线运动 |
| 圆弧运动   | `moveCircular()` | 笛卡尔圆弧运动 |
| 相对运动   | `moveRelative()` | 相对位姿运动  |
| Jog关节  | `jogJoint()`     | 手动关节运动  |
| Jog笛卡尔 | `jogCartesian()` | 手动TCP运动 |

---

# 11.6 运动规划功能

| 功能名称  | 接口                     | 功能说明    |
| ----- | ---------------------- | ------- |
| 规划到位姿 | `planToPose()`         | 规划到目标位姿 |
| 规划到关节 | `planToJoint()`        | 规划到关节位置 |
| 多路点规划 | `planWaypoints()`      | 多点路径规划  |
| 规划并执行 | `planAndExecute()`     | 规划后立即执行 |
| 路径验证  | `validateTrajectory()` | 验证轨迹合法性 |

---

# 11.7 轨迹执行控制

| 功能名称   | 接口                       | 功能说明     |
| ------ | ------------------------ | -------- |
| 执行轨迹   | `executeTrajectory()`    | 执行规划轨迹   |
| 暂停执行   | `pauseExecution()`       | 暂停当前轨迹   |
| 恢复执行   | `resumeExecution()`      | 恢复执行     |
| 取消执行   | `cancelExecution()`      | 终止当前轨迹   |
| 查询执行进度 | `getExecutionProgress()` | 获取轨迹执行进度 |

---

# 11.8 规划场景管理

| 功能名称    | 接口                      | 功能说明    |
| ------- | ----------------------- | ------- |
| 初始化场景   | `initializeScene()`     | 初始化规划场景 |
| 添加立方体障碍 | `addBoxObstacle()`      | 添加立方体   |
| 添加圆柱体障碍 | `addCylinderObstacle()` | 添加圆柱体   |
| 添加网格模型  | `addMeshObstacle()`     | 添加三维模型  |
| 删除障碍物   | `removeObstacle()`      | 删除指定障碍物 |
| 更新障碍物   | `updateObstaclePose()`  | 更新障碍物位置 |
| 清空场景    | `clearScene()`          | 清空规划环境  |
| 保存场景    | `saveScene()`           | 保存规划场景  |
| 加载场景    | `loadScene()`           | 加载规划场景  |

---

# 11.9 焊接任务控制

| 功能名称    | 接口                        | 功能说明    |
| ------- | ------------------------- | ------- |
| 移动到焊接起点 | `moveToWeldStart()`       | 到达焊接起始点 |
| 执行焊接轨迹  | `executeWeldTrajectory()` | 执行焊接路径  |
| 暂停焊接    | `pauseWelding()`          | 暂停焊接    |
| 恢复焊接    | `resumeWelding()`         | 恢复焊接    |
| 停止焊接    | `stopWelding()`           | 停止焊接    |

---

# 11.10 焊接工艺控制

| 功能名称   | 接口                    | 功能说明    |
| ------ | --------------------- | ------- |
| 起弧     | `startArc()`          | 启动焊接    |
| 停弧     | `stopArc()`           | 停止焊接    |
| 设置焊接参数 | `setWeldParameters()` | 设置焊接电流等 |
| 获取焊接参数 | `getWeldParameters()` | 查询焊接参数  |

---

# 11.11 摆焊控制

| 功能名称   | 接口                       | 功能说明   |
| ------ | ------------------------ | ------ |
| 启用摆焊   | `enableWeaving()`        | 开启摆焊   |
| 关闭摆焊   | `disableWeaving()`       | 关闭摆焊   |
| 设置摆焊参数 | `setWeavingParameters()` | 设置摆焊参数 |
| 获取摆焊参数 | `getWeavingParameters()` | 查询摆焊参数 |

---

# 11.12 系统安全控制

| 功能名称   | 接口                            | 功能说明   |
| ------ | ----------------------------- | ------ |
| 急停     | `emergencyStop()`             | 紧急停止   |
| 碰撞检测   | `enableCollisionDetection()`  | 启用碰撞保护 |
| 关闭碰撞检测 | `disableCollisionDetection()` | 关闭碰撞保护 |
| 安全区域设置 | `setSafetyZone()`             | 设置安全区域 |

---

# 11.13 日志与诊断

| 功能名称   | 接口                   | 功能说明   |
| ------ | -------------------- | ------ |
| 系统日志记录 | `logSystemEvent()`   | 记录系统事件 |
| 错误日志查询 | `getErrorLog()`      | 查询错误记录 |
| 轨迹日志   | `getTrajectoryLog()` | 获取轨迹记录 |
| 系统诊断   | `runDiagnostics()`   | 运行系统诊断 |

---

# 11.14 功能统计

| 类别      | 功能数量 |
| ------- | ---- |
| 系统管理    | 4    |
| 机器人基础控制 | 6    |
| 初始化运动   | 4    |
| 状态查询    | 5    |
| 运动控制    | 6    |
| 规划功能    | 5    |
| 轨迹执行    | 5    |
| 场景管理    | 9    |
| 焊接控制    | 5    |
| 焊接工艺    | 4    |
| 摆焊控制    | 4    |
| 安全控制    | 4    |
| 日志诊断    | 4    |

**总功能数量：65+**

---

# 12 接口分级设计

为了提高系统的可维护性与可扩展性，CR7 Robot Controller 对外提供的接口按照功能与抽象层级进行分级设计。
不同层级的接口承担不同职责，并建议用户按照层级顺序进行调用。

接口分级结构如下：

```
Level 1  系统级接口
Level 2  机器人控制接口
Level 3  运动规划接口
Level 4  任务应用接口
```

各级接口说明如下。

---

# 12.1 Level 1 系统级接口（System Level API）

系统级接口负责 **系统生命周期管理、安全控制以及基础状态管理**。
这些接口是整个机器人系统运行的基础，通常在系统启动和关闭阶段使用。

| 接口                 | 功能说明       |
| ------------------ | ---------- |
| initializeSystem() | 初始化机器人控制系统 |
| shutdownSystem()   | 关闭机器人控制系统  |
| powerOn()          | 机器人控制柜上电   |
| powerOff()         | 机器人控制柜下电   |
| enableRobot()      | 机器人进入可运动状态 |
| disableRobot()     | 机器人禁止运动    |
| resetRobot()       | 清除机器人错误状态  |
| emergencyStop()    | 紧急停止机器人    |
| getSystemState()   | 获取系统整体状态   |

**特点**

* 系统级控制
* 安全优先
* 不涉及具体运动

---

# 12.2 Level 2 机器人控制接口（Robot Motion API）

该层接口用于控制机器人进行 **基础运动操作**。
主要包括机器人运动、状态获取以及基础控制。

| 接口                  | 功能说明     |
| ------------------- | -------- |
| moveJoint()         | 关节空间运动   |
| moveLinear()        | 笛卡尔直线运动  |
| moveCircular()      | 圆弧运动     |
| moveRelative()      | 相对位姿运动   |
| jogJoint()          | 关节Jog运动  |
| jogCartesian()      | 笛卡尔Jog运动 |
| moveToHome()        | 回关节零点    |
| moveToInitialPose() | 回初始位置    |
| moveToSafePose()    | 移动到安全位姿  |
| stopMotion()        | 停止当前运动   |

**特点**

* 提供机器人 **直接运动能力**
* 不涉及路径规划
* 常用于 **手动操作 / 调试**

---

# 12.3 Level 3 运动规划接口（Motion Planning API）

该层接口提供 **路径规划与轨迹执行能力**。
主要用于复杂任务的自动路径生成与执行。

| 接口                   | 功能说明    |
| -------------------- | ------- |
| planToPose()         | 规划到目标位姿 |
| planToJoint()        | 规划到目标关节 |
| planWaypoints()      | 多路点轨迹规划 |
| planAndExecute()     | 规划并执行   |
| executeTrajectory()  | 执行规划轨迹  |
| pauseExecution()     | 暂停轨迹执行  |
| resumeExecution()    | 恢复轨迹执行  |
| cancelExecution()    | 取消轨迹执行  |
| validateTrajectory() | 验证轨迹合法性 |

**特点**

* 使用 **路径规划算法（OMPL等）**
* 支持 **避障**
* 支持 **复杂路径任务**

---

# 12.4 Level 4 任务应用接口（Application API）

该层接口针对 **焊接应用场景**进行封装。
用户无需关注底层运动规划，只需调用应用接口即可完成任务。

| 接口                      | 功能说明    |
| ----------------------- | ------- |
| moveToWeldStart()       | 移动到焊接起点 |
| executeWeldTrajectory() | 执行焊接轨迹  |
| pauseWelding()          | 暂停焊接    |
| resumeWelding()         | 恢复焊接    |
| stopWelding()           | 停止焊接    |
| startArc()              | 启动焊接电弧  |
| stopArc()               | 停止焊接电弧  |
| setWeldParameters()     | 设置焊接参数  |
| enableWeaving()         | 启用摆焊    |
| setWeavingParameters()  | 设置摆焊参数  |

**特点**

* 面向 **具体工业应用**
* 封装底层复杂逻辑
* 用户调用最简单

---

# 12.5 接口调用关系

系统接口的调用关系建议如下：

```
Application API
      ↓
Motion Planning API
      ↓
Robot Motion API
      ↓
System API
```

即：

```
焊接任务
  ↓
路径规划
  ↓
机器人运动
  ↓
系统控制
```

---

# 12.6 接口使用建议

为了保证系统稳定性，建议按照以下顺序调用接口：

```
1 系统初始化
2 机器人使能
3 规划场景配置
4 路径规划
5 轨迹执行
6 焊接任务执行
7 系统关闭
```

推荐的典型流程如下：

```
initializeSystem()
enableRobot()

initializeScene()

moveToInitialPose()

moveToWeldStart()

executeWeldTrajectory()

stopArc()

moveToSafePose()

shutdownSystem()
```

---

# 总结

CR7 Robot Controller 采用 **模块化分层架构**，核心特点包括：

* 规划与执行完全分离
* 提供统一用户 API
* 支持规划场景管理
* 支持焊接专用控制
* 支持系统初始化与安全恢复

系统具备良好的 **稳定性、可维护性与扩展能力**。

---

