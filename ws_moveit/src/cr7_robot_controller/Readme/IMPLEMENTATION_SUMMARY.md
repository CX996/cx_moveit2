# PILZ焊接路径轨迹记录和分析系统

## 📋 项目概述

为CR7机器人的PILZ工业规划器焊接路径测试增加了完整的轨迹记录和分析功能。用户可以：
1. ✅ 执行PILZ焊接路径测试
2. ✅ 自动记录规划轨迹到CSV文件
3. ✅ 使用专门的分析工具检查路径直线性
4. ✅ 生成详细的分析报告和图表

---

## 🔧 代码修改

### 1. **C++代码修改** (`cr7_robot_controller.cpp`)

#### a) 修改 `executePilzPlan()` 方法 (第786-788行)
添加了轨迹分析保存功能：
```cpp
// 保存轨迹分析信息
std::string trajectory_prefix = "pilz_" + planner_id + "_trajectory";
saveTrajectoryAnalysis(plan.trajectory_, trajectory_prefix);
```

**功能**: 将PILZ规划器生成的关节轨迹保存到CSV文件
**文件格式**: `pilz_LIN_trajectory_YYYYMMDD_HHMMSS.csv`
**包含数据**: 关节位置、速度、加速度、时间戳

#### b) 修改 `executePilzWeldingPath()` 方法 (第1732-1739行)
添加了焊接路径点分析保存功能：
```cpp
// 保存焊接路径点信息
std::vector<geometry_msgs::msg::Pose> welding_waypoints = {
    start_wp.toPose(),
    target_pose
};
saveWaypointAnalysis(welding_waypoints, "pilz_welding_waypoints");
```

**功能**: 保存焊接起点和终点坐标到CSV文件
**文件格式**: `pilz_welding_waypoints_YYYYMMDD_HHMMSS.csv`
**包含数据**: 坐标(x,y,z)、四元数、点间距、累积距离

#### c) 头文件声明 (`cr7_robot_controller.hpp`)
添加了新的公共方法：
```cpp
Result executePilzWeldingPath();
```

---

## 📊 Python分析工具

### 文件: `analyze_pilz_welding.py`

一个专门用于分析PILZ焊接路径的Python脚本，提供以下功能：

#### 主要类: `PilzWeldingAnalyzer`

##### 方法列表

| 方法 | 功能 | 输出 |
|------|------|------|
| `load_pilz_data()` | 加载焊接路径和轨迹数据 | CSV文件加载状态 |
| `analyze_welding_waypoints()` | 分析焊接路径点 | 直线性统计 |
| `analyze_pilz_trajectory()` | 分析PILZ轨迹 | 运动学特性 |
| `plot_pilz_welding_analysis()` | 生成焊接分析图表 | `pilz_welding_analysis.png` |
| `plot_linearity_check()` | 生成直线性检查图表 | `pilz_linearity_check.png` |
| `generate_report()` | 生成文本报告 | `pilz_welding_analysis_report.txt` |

#### 生成的图表

**pilz_welding_analysis.png** (6个子图)
- 3D焊接路径可视化
- XY平面投影
- 路径点间距分布
- 轨迹点时间间隔
- 关节速度曲线
- 关节加速度曲线

**pilz_linearity_check.png** (6个子图)
- 3D直线性检查（实际 vs 理想）
- XY/XZ平面投影对比
- 总体偏差分布
- 各轴偏差对比
- 偏差直方图

---

## 📁 数据流

```
机器人执行PILZ焊接路径
        ↓
saveTrajectoryAnalysis() ← 保存关节轨迹
saveWaypointAnalysis()   ← 保存路径点
        ↓
CSV文件生成
        ├─ pilz_welding_waypoints_*.csv
        └─ pilz_LIN_trajectory_*.csv
        ↓
analyze_pilz_welding.py
        ↓
输出分析结果
        ├─ pilz_welding_analysis.png
        ├─ pilz_linearity_check.png
        └─ pilz_welding_analysis_report.txt
```

---

## 🚀 使用流程

### 第1步: 运行机器人控制程序
```bash
cd ~/cx_moveit/ws_moveit
source install/setup.bash
ros2 launch cr7_robot cr7_controller.launch.py
```

### 第2步: 执行PILZ焊接路径测试
在主菜单选择:
- `9` - PILZ规划器测试
- `6` - PILZ焊接点位路径测试

机器人将自动:
1. 使用关节空间规划移动到起点
2. 使用PILZ LIN规划器执行直线运动到终点
3. 自动保存轨迹数据到CSV文件

### 第3步: 分析轨迹数据
```bash
cd ~/cx_moveit/ws_moveit/src/cr7_robot/data

# 方法1: 使用快速启动脚本
./run_pilz_analysis.sh

# 方法2: 直接运行Python脚本
python3 analyze_pilz_welding.py
```

### 第4步: 查看结果
```bash
# 查看生成的图表
eog pilz_welding_analysis.png &
eog pilz_linearity_check.png &

# 查看文本报告
cat pilz_welding_analysis_report.txt
```

---

## 📈 关键指标说明

### 直线性系数 (Linearity Coefficient)
```
直线性系数 = 直线距离 / 路径长度
```

| 范围 | 含义 | 建议 |
|------|------|------|
| > 0.99 | ✓ 完全直线 | 正常 |
| 0.95-0.99 | ⚠ 接近直线 | 可接受 |
| < 0.95 | ✗ 明显弯曲 | 需要优化 |

### 偏差统计
- **最大偏差**: 路径与理想直线的最大距离（mm）
- **平均偏差**: 所有点的平均偏离距离（mm）
- **标准差**: 偏差的波动程度

### 运动学特性
- **最大速度**: 关节在轨迹中达到的最大角速度
- **最大加速度**: 关节在轨迹中达到的最大角加速度
- **执行时间**: PILZ规划器执行轨迹的总时间

---

## 📝 输出文件格式

### 焊接路径点CSV (`pilz_welding_waypoints_*.csv`)
```
index,x,y,z,qx,qy,qz,qw,distance_from_prev,cumulative_distance
0,0.977720,-0.374100,0.028286,0.539280,0.817230,0.027122,0.201420,0.000000,0.000000
1,0.977720,0.377530,0.028286,-0.373310,0.899200,-0.084073,0.212170,0.751630,0.751630
```

### PILZ轨迹CSV (`pilz_LIN_trajectory_*.csv`)
```
index,time_from_start,position_0(joint_name),position_1(...),velocity_0(...),acceleration_0(...),etc
0,0.000000,0.123456,0.234567,0.000000,0.000000,...
1,0.010000,0.123456,0.235000,0.043000,0.430000,...
...
```

---

## 🔍 常见分析场景

### 场景1: 检查路径是否为直线
```bash
# 运行分析
./run_pilz_analysis.sh

# 查看直线性系数
grep "直线性系数" pilz_welding_analysis_report.txt
grep "结论" pilz_welding_analysis_report.txt
```

### 场景2: 比较PILZ vs 笛卡尔规划器
```bash
# 查看对比统计
grep -A5 "PILZ vs 笛卡尔" pilz_welding_analysis_report.txt
```

### 场景3: 分析运动学特性
```bash
# 查看关节速度和加速度
grep -A3 "运动学特性" pilz_welding_analysis_report.txt
```

---

## 📋 依赖项

### C++编译时
- moveit2 (已有)
- rclcpp (已有)

### Python运行时
```bash
pip3 install pandas numpy matplotlib
```

---

## 📚 文件清单

### 新增/修改的C++代码
- ✅ `src/cr7_robot_controller.cpp` - 修改(+轨迹保存)
- ✅ `include/cr7_robot_controller/cr7_robot_controller.hpp` - 修改(+方法声明)

### 新增Python工具
- ✅ `data/analyze_pilz_welding.py` - 新增(PILZ分析工具)
- ✅ `data/run_pilz_analysis.sh` - 新增(快速启动脚本)
- ✅ `data/PILZ_ANALYSIS_README.md` - 新增(使用文档)

### 生成的数据文件
- 📊 `data/pilz_welding_waypoints_*.csv` - 焊接路径点
- 📊 `data/pilz_LIN_trajectory_*.csv` - PILZ轨迹
- 🖼️ `data/pilz_welding_analysis.png` - 分析图表
- 🖼️ `data/pilz_linearity_check.png` - 直线性检查图
- 📄 `data/pilz_welding_analysis_report.txt` - 文本报告

---

## 💡 优化建议

### 如果路径不是直线
1. 检查PILZ规划参数
2. 增加路径点数量
3. 调整速度/加速度因子
4. 检查机器人运动学约束

### 如果执行时间过长
1. 增加最大速度因子
2. 减少轨迹点数
3. 检查规划时间设置

### 如果出现抖动
1. 增加加速度因子上限
2. 平滑化轨迹
3. 检查机器人硬件状态

---

## 🐛 故障排除

### Q: 找不到CSV文件？
A: 确保执行了PILZ焊接路径测试（选择选项6）

### Q: Python脚本运行出错？
A: 运行 `pip3 install pandas numpy matplotlib`

### Q: 图表显示中文为方块？
A: 安装中文字体：`sudo apt-get install fonts-wqy-zenhei`

### Q: 如何自定义焊接路径？
A: 修改 `executePilzWeldingPath()` 中的起点和终点坐标

---

## 📞 反馈和改进

如有问题或建议，请检查：
- 轨迹CSV数据是否完整
- Python依赖是否正确安装
- 系统是否有中文字体支持

---

## ✅ 总结

这个系统提供了完整的PILZ焊接路径验证解决方案：

1. **自动记录**: C++代码自动保存轨迹数据
2. **智能分析**: Python工具详细分析路径特性
3. **可视化**: 生成多维度的分析图表
4. **易用性**: 提供快速启动脚本和详细文档

用户可以快速验证PILZ规划器的执行效果，并进行持续优化！
