# OMPL规划器代码重构计划

## 目标
重构OMPL规划器代码，实现约束设置和执行的分离，为不同约束类型创建专门的执行函数，并优化测试代码结构。

## 重构步骤

### 1. OMPL规划器重构

#### 1.1 约束执行函数
- 创建`moveWithBoxConstraint`函数：执行带盒子约束的规划
- 创建`moveWithPlaneConstraint`函数：执行带平面约束的规划
- 创建`moveWithLineConstraint`函数：执行带直线约束的规划
- 创建`moveWithOrientationConstraint`函数：执行带姿态约束的规划

#### 1.2 直线性检查
- 在`moveWithLineConstraint`函数中集成轨迹直线性检查
- 使用`TrajectoryAnalyzer::isTrajectoryLinear`方法验证轨迹是否为直线
- 如果轨迹不是直线，重新尝试规划

#### 1.3 代码结构优化
- 保持现有的约束设置方法不变
- 新的执行函数将内部处理约束设置和清理
- 确保与现有接口兼容

### 2. 测试代码优化

#### 2.1 路径执行器扩展
- 在`CR7PathExecutor`类中添加OMPL约束规划测试方法：
  - `executeOMPLBoxConstraintTest`
  - `executeOMPLPlaneConstraintTest`
  - `executeOMPLLineConstraintTest`
  - `executeOMPLOrientationConstraintTest`
  - `executeOMPLConstraintTest`（整合所有测试）

#### 2.2 主控制器接口
- 在`CR7RobotController`类中添加对应的接口方法：
  - `executeOMPLBoxConstraintTest`
  - `executeOMPLPlaneConstraintTest`
  - `executeOMPLLineConstraintTest`
  - `executeOMPLOrientationConstraintTest`
  - `executeOMPLConstraintTest`

#### 2.3 测试代码清理
- 从`main.cpp`中移除OMPL约束规划测试代码
- 更新`main.cpp`，使用新的控制器接口

## 技术细节

### 约束执行函数设计
每个约束执行函数将：
1. 设置对应的约束
2. 执行规划
3. 检查轨迹（对于直线约束）
4. 执行运动
5. 清理约束