#!/usr/bin/env python3
"""
PILZ焊接路径分析工具
用于分析CR7机器人使用PILZ规划器执行的焊接路径
比较原始路径点和实际规划轨迹的偏差
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
import glob
import os
import sys
from datetime import datetime
from matplotlib import font_manager

# 尝试导入KDL用于正向运动学计算
try:
    import PyKDL as kdl
    HAS_KDL = True
except ImportError:
    HAS_KDL = False
    print("⚠ 警告: 未找到PyKDL, 正向运动学计算将不可用")


# ================================
# 中文字体安全加载
# ================================
def set_chinese_font():
    """
    安全设置中文字体
    """
    candidate_fonts = [
        "/usr/share/fonts/truetype/arphic/ukai.ttc",
        "/usr/share/fonts/truetype/arphic/uming.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    ]

    font_path = None
    for f in candidate_fonts:
        if os.path.exists(f):
            font_path = f
            break

    if font_path:
        font_prop = font_manager.FontProperties(fname=font_path)
        plt.rcParams['font.family'] = font_prop.get_name()
        print(f"✓ 已设置中文字体: {font_prop.get_name()}")
    else:
        print("⚠ 警告: 未找到可用中文字体")

    plt.rcParams['axes.unicode_minus'] = False


set_chinese_font()


class PilzWeldingAnalyzer:
    """PILZ焊接路径分析器"""
    
    def __init__(self, data_dir="."):
        """
        初始化分析器
        
        Args:
            data_dir: 数据文件目录
        """
        self.data_dir = data_dir
        self.waypoints_data = None           # 焊接路径点 (起点->终点)
        self.trajectory_data = None         # PILZ规划的轨迹
        self.cartesian_trajectory = None    # 笛卡尔轨迹（用于对比）
        
    def load_pilz_data(self):
        """
        加载PILZ焊接路径数据
        """
        print("\n" + "="*60)
        print("加载PILZ焊接路径数据")
        print("="*60)
        
        # 查找焊接路径点文件
        waypoint_files = glob.glob(os.path.join(self.data_dir, "pilz_welding_waypoints_*.csv"))
        if waypoint_files:
            latest_waypoint = max(waypoint_files, key=os.path.getctime)
            self.waypoints_data = pd.read_csv(latest_waypoint)
            print(f"✓ 已加载焊接路径点: {os.path.basename(latest_waypoint)}")
            print(f"  数据形状: {self.waypoints_data.shape}")
        else:
            print("✗ 未找到焊接路径点文件")
        
        # 查找PILZ LIN轨迹文件
        lin_files = glob.glob(os.path.join(self.data_dir, "pilz_LIN_trajectory_*.csv"))
        if lin_files:
            latest_trajectory = max(lin_files, key=os.path.getctime)
            self.trajectory_data = pd.read_csv(latest_trajectory)
            print(f"✓ 已加载PILZ LIN轨迹: {os.path.basename(latest_trajectory)}")
            print(f"  数据形状: {self.trajectory_data.shape}")
        else:
            print("✗ 未找到PILZ LIN轨迹文件")
        
        # 查找笛卡尔轨迹用于对比
        cart_files = glob.glob(os.path.join(self.data_dir, "original_trajectory_*.csv"))
        if cart_files:
            latest_cart = max(cart_files, key=os.path.getctime)
            self.cartesian_trajectory = pd.read_csv(latest_cart)
            print(f"✓ 已加载笛卡尔轨迹（对比用）: {os.path.basename(latest_cart)}")
        else:
            print("! 笛卡尔轨迹文件不可用（可跳过）")
    
    def analyze_welding_waypoints(self):
        """
        分析焊接路径点
        """
        if self.waypoints_data is None:
            print("✗ 没有可用的焊接路径点数据")
            return
        
        print("\n" + "="*60)
        print("焊接路径点分析")
        print("="*60)
        
        data = self.waypoints_data
        
        print(f"总路径点数: {len(data)}")
        print(f"总路径长度: {data['cumulative_distance'].iloc[-1]:.6f} m")
        
        if len(data) >= 2:
            start_point = data.iloc[0]
            end_point = data.iloc[-1]
            
            print(f"\n起点: [{start_point['x']:.6f}, {start_point['y']:.6f}, {start_point['z']:.6f}]")
            print(f"终点: [{end_point['x']:.6f}, {end_point['y']:.6f}, {end_point['z']:.6f}]")
            
            # 计算直线距离
            straight_distance = np.sqrt(
                (end_point['x'] - start_point['x'])**2 +
                (end_point['y'] - start_point['y'])**2 +
                (end_point['z'] - start_point['z'])**2
            )
            print(f"直线距离: {straight_distance:.6f} m")
            
            # 计算路径是否为直线
            path_length = data['cumulative_distance'].iloc[-1]
            linearity = straight_distance / path_length if path_length > 0 else 0
            print(f"直线性系数: {linearity:.4f} (1.0=完全直线)")
            
            if linearity > 0.99:
                print("✓ 路径是直线")
            elif linearity > 0.95:
                print("⚠ 路径接近直线，存在轻微弯曲")
            else:
                print("✗ 路径有明显弯曲")
    
    def analyze_pilz_trajectory(self):
        """
        分析PILZ规划的轨迹
        """
        if self.trajectory_data is None:
            print("✗ 没有可用的PILZ轨迹数据")
            return
        
        print("\n" + "="*60)
        print("PILZ LIN轨迹分析")
        print("="*60)
        
        data = self.trajectory_data
        
        print(f"轨迹点数: {len(data)}")
        print(f"轨迹总时间: {data['time_from_start'].iloc[-1]:.3f} 秒")
        
        # 查找关节列
        position_cols = [col for col in data.columns if col.startswith('position_')]
        velocity_cols = [col for col in data.columns if col.startswith('velocity_')]
        acceleration_cols = [col for col in data.columns if col.startswith('acceleration_')]
        
        print(f"关节数量: {len(position_cols)}")
        
        # 计算平均时间步长
        time_steps = np.diff(data['time_from_start'])
        print(f"\n时间步长统计:")
        print(f"  平均: {time_steps.mean():.6f} 秒")
        print(f"  最大: {time_steps.max():.6f} 秒")
        print(f"  最小: {time_steps.min():.6f} 秒")
        
        # 分析速度和加速度
        print(f"\n运动学特性:")
        for i, pos_col in enumerate(position_cols[:3]):  # 只显示前3个关节
            joint_name = pos_col.split('(')[-1].rstrip(')') if '(' in pos_col else f"J{i}"
            
            # 速度分析
            if i < len(velocity_cols):
                vel_col = velocity_cols[i]
                if vel_col in data.columns:
                    velocities = data[vel_col].abs()
                    print(f"\n  {joint_name}:")
                    print(f"    最大速度: {velocities.max():.4f} rad/s")
                    print(f"    平均速度: {velocities.mean():.4f} rad/s")
            
            # 加速度分析
            if i < len(acceleration_cols):
                acc_col = acceleration_cols[i]
                if acc_col in data.columns:
                    accelerations = data[acc_col].abs()
                    print(f"    最大加速度: {accelerations.max():.4f} rad/s²")
                    print(f"    平均加速度: {accelerations.mean():.4f} rad/s²")
    
    def calculate_end_effector_trajectory(self):
        """
        从关节轨迹计算末端执行器轨迹
        使用正向运动学 (FK) 转换关节角度为笛卡尔坐标
        """
        if self.trajectory_data is None:
            print("✗ 没有可用的轨迹数据")
            return None
        
        print("\n正在计算末端执行器轨迹...")
        
        if not HAS_KDL:
            print("⚠ 警告: 未安装PyKDL，使用简化的FK计算（仅用于演示）")
            return self._calculate_ee_trajectory_simple()
        
        try:
            # 使用KDL计算FK
            return self._calculate_ee_trajectory_kdl()
        except Exception as e:
            print(f"⚠ KDL计算失败: {e}，尝试简化方法...")
            return self._calculate_ee_trajectory_simple()
    
    def _calculate_ee_trajectory_kdl(self):
        """
        使用PyKDL计算末端执行器轨迹
        """
        # CR7机器人的标准DH参数（需要根据实际机器人调整）
        # 这是一个标准的6轴机器人配置
        chain = kdl.Chain()
        
        # 添加关节和连杆（这里使用简化的模型）
        # a, d, alpha, offset (单位: m, rad)
        dh_params = [
            (0.0, 0.4, np.pi/2, 0.0),
            (0.425, 0.0, 0.0, -np.pi/2),
            (0.39225, 0.0, 0.0, 0.0),
            (0.0, 0.40925, np.pi/2, 0.0),
            (0.0, 0.0, -np.pi/2, 0.0),
            (0.0, 0.109, 0.0, 0.0),
        ]
        
        # 构建运动链
        for a, d, alpha, offset in dh_params:
            frame = kdl.Frame(
                kdl.Rotation.RotZ(0),
                kdl.Vector(a, 0, d)
            )
            chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ), frame))
        
        # 创建FK求解器
        fksolver = kdl.ChainFkSolverPos_recursive(chain)
        
        # 提取关节列
        joint_cols = [col for col in self.trajectory_data.columns if col.startswith('position_')]
        
        ee_trajectory = []
        
        for idx, row in self.trajectory_data.iterrows():
            try:
                # 读取关节角度
                joint_angles = [row[col] for col in joint_cols[:6]]
                
                # 创建关节数组
                q = kdl.JntArray(6)
                for i, angle in enumerate(joint_angles):
                    q[i] = angle
                
                # 计算FK
                frame = kdl.Frame()
                status = fksolver.JntToCart(q, frame)
                
                if status >= 0:
                    x, y, z = frame.p.x(), frame.p.y(), frame.p.z()
                    ee_trajectory.append({'x': x, 'y': y, 'z': z})
                else:
                    print(f"✗ 行{idx}的FK计算失败")
                    
            except Exception as e:
                print(f"✗ 行{idx}出错: {e}")
        
        if ee_trajectory:
            result = pd.DataFrame(ee_trajectory)
            print(f"✓ 成功计算{len(result)}个末端位置点")
            return result
        else:
            print("⚠ FK计算失败，没有有效的末端轨迹")
            return None
    
    def _calculate_ee_trajectory_simple(self):
        """
        简化的FK计算（基于直接关节-笛卡尔映射）
        用于当PyKDL不可用时的后备方案
        """
        print("使用简化FK模型进行计算...")
        
        # 提取关节列
        joint_cols = [col for col in self.trajectory_data.columns if col.startswith('position_')]
        
        if not joint_cols:
            print("✗ 未找到关节位置列")
            return None
        
        ee_trajectory = []
        
        # 使用近似的正向运动学
        # 这是一个简化模型，实际应使用完整的DH参数
        for idx, row in self.trajectory_data.iterrows():
            try:
                # 假设关节0-2决定位置，关节3-5决定姿态
                j0 = row[joint_cols[0]] if len(joint_cols) > 0 else 0
                j1 = row[joint_cols[1]] if len(joint_cols) > 1 else 0
                j2 = row[joint_cols[2]] if len(joint_cols) > 2 else 0
                
                # 简化的FK（这只是演示，需要真实的DH参数）
                # 实际值应根据CR7的具体参数计算
                L1, L2 = 0.425, 0.39225  # 臂长（m）
                
                x = L1 * np.cos(j0) * np.cos(j1) + L2 * np.cos(j0) * np.cos(j1 + j2)
                y = L1 * np.sin(j0) * np.cos(j1) + L2 * np.sin(j0) * np.cos(j1 + j2)
                z = 0.4 + L1 * np.sin(j1) + L2 * np.sin(j1 + j2)
                
                ee_trajectory.append({'x': x, 'y': y, 'z': z})
                
            except Exception as e:
                print(f"✗ 行{idx}出错: {e}")
        
        if ee_trajectory:
            result = pd.DataFrame(ee_trajectory)
            print(f"✓ 使用简化模型计算了{len(result)}个末端位置点")
            print("⚠ 注意: 这是简化计算，精度可能不高")
            return result
        else:
            return None
    
    def compare_with_cartesian(self):
        """
        与笛卡尔轨迹对比
        """
        if self.cartesian_trajectory is None:
            print("✗ 没有可用的笛卡尔轨迹用于对比")
            return
        
        print("\n" + "="*60)
        print("PILZ vs 笛卡尔轨迹对比")
        print("="*60)
        
        pilz_len = len(self.trajectory_data) if self.trajectory_data is not None else 0
        cart_len = len(self.cartesian_trajectory)
        
        if pilz_len > 0:
            print(f"PILZ轨迹点数: {pilz_len}")
            print(f"笛卡尔轨迹点数: {cart_len}")
            
            if cart_len > 0:
                reduction = (cart_len - pilz_len) / cart_len * 100
                print(f"点数减少: {reduction:.1f}%")
            
            pilz_time = self.trajectory_data['time_from_start'].iloc[-1]
            cart_time = self.cartesian_trajectory['time_from_start'].iloc[-1]
            
            print(f"\nPILZ执行时间: {pilz_time:.3f} 秒")
            print(f"笛卡尔执行时间: {cart_time:.3f} 秒")
            
            time_diff = (pilz_time - cart_time) / cart_time * 100 if cart_time > 0 else 0
            print(f"时间差异: {time_diff:+.1f}%")
    
    def plot_pilz_welding_analysis(self):
        """
        绘制PILZ焊接路径分析图表
        """
        if self.waypoints_data is None or self.trajectory_data is None:
            print("✗ 缺少必要的数据")
            return
        
        # 计算末端执行器轨迹 (从关节角度转换为笛卡尔坐标)
        print("计算末端执行器轨迹...")
        ee_trajectory = self.calculate_end_effector_trajectory()
        
        fig = plt.figure(figsize=(18, 12))
        
        # 1. 焊接路径点3D（期望路径）
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        
        wp_data = self.waypoints_data
        ax1.plot(wp_data['x'], wp_data['y'], wp_data['z'], 'g-', linewidth=3, label='期望路径')
        ax1.scatter(wp_data['x'], wp_data['y'], wp_data['z'], c='g', s=100, marker='o', alpha=0.6)
        ax1.scatter(wp_data['x'].iloc[0], wp_data['y'].iloc[0], wp_data['z'].iloc[0], 
                   c='blue', s=200, marker='o', label='起点')
        ax1.scatter(wp_data['x'].iloc[-1], wp_data['y'].iloc[-1], wp_data['z'].iloc[-1], 
                   c='red', s=200, marker='s', label='终点')
        
        # 如果有末端执行器轨迹，也绘制实际执行的轨迹
        if ee_trajectory is not None and len(ee_trajectory) > 0:
            ax1.plot(ee_trajectory['x'], ee_trajectory['y'], ee_trajectory['z'], 
                    'b--', linewidth=2, alpha=0.7, label='实际轨迹')
            ax1.scatter(ee_trajectory['x'].iloc[::max(1, len(ee_trajectory)//10)], 
                       ee_trajectory['y'].iloc[::max(1, len(ee_trajectory)//10)], 
                       ee_trajectory['z'].iloc[::max(1, len(ee_trajectory)//10)],
                       c='b', s=30, marker='^', alpha=0.5)
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('PILZ焊接路径对比')
        ax1.legend()
        ax1.grid(True)
        
        # 2. 焊接路径点XY投影
        ax2 = fig.add_subplot(2, 3, 2)
        ax2.plot(wp_data['x'], wp_data['y'], 'g-', linewidth=2, label='期望路径')
        ax2.scatter(wp_data['x'], wp_data['y'], c='g', s=50, alpha=0.6)
        ax2.scatter(wp_data['x'].iloc[0], wp_data['y'].iloc[0], c='blue', s=150, marker='o')
        ax2.scatter(wp_data['x'].iloc[-1], wp_data['y'].iloc[-1], c='red', s=150, marker='s')
        
        # 绘制实际轨迹
        if ee_trajectory is not None and len(ee_trajectory) > 0:
            ax2.plot(ee_trajectory['x'], ee_trajectory['y'], 'b--', linewidth=2, alpha=0.7, label='实际轨迹')
        
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title('XY平面投影对比')
        ax2.grid(True)
        ax2.axis('equal')
        ax2.legend()
        
        # 3. 焊接路径点距离分析
        ax3 = fig.add_subplot(2, 3, 3)
        distances = wp_data['distance_from_prev'][1:]
        ax3.bar(range(len(distances)), distances, alpha=0.7, color='g')
        ax3.axhline(y=distances.mean(), color='r', linestyle='--', label=f'平均值: {distances.mean():.4f}m')
        ax3.set_xlabel('路径段索引')
        ax3.set_ylabel('距离 (m)')
        ax3.set_title('焊接路径点间距分析')
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        
        # 4. PILZ轨迹点数分布
        ax4 = fig.add_subplot(2, 3, 4)
        traj_data = self.trajectory_data
        time_intervals = np.diff(traj_data['time_from_start'])
        ax4.plot(traj_data['time_from_start'].iloc[1:], time_intervals * 1000, 'b-', linewidth=1.5, alpha=0.7)
        ax4.axhline(y=np.mean(time_intervals)*1000, color='r', linestyle='--', 
                   label=f'平均: {np.mean(time_intervals)*1000:.2f}ms')
        ax4.set_xlabel('时间 (s)')
        ax4.set_ylabel('时间间隔 (ms)')
        ax4.set_title('PILZ轨迹时间间隔')
        ax4.grid(True, alpha=0.3)
        ax4.legend()
        
        # 5. 关节速度对比
        ax5 = fig.add_subplot(2, 3, 5)
        position_cols = [col for col in traj_data.columns if col.startswith('position_')]
        velocity_cols = [col for col in traj_data.columns if col.startswith('velocity_')]
        
        for i in range(min(3, len(position_cols))):
            if i < len(velocity_cols):
                joint_name = position_cols[i].split('(')[-1].rstrip(')') if '(' in position_cols[i] else f"J{i}"
                ax5.plot(traj_data['time_from_start'], 
                        traj_data[velocity_cols[i]], 
                        label=joint_name, linewidth=1.5, alpha=0.8)
        
        ax5.set_xlabel('时间 (s)')
        ax5.set_ylabel('速度 (rad/s)')
        ax5.set_title('关节速度曲线（前3个关节）')
        ax5.grid(True, alpha=0.3)
        ax5.legend(loc='best', fontsize=8)
        
        # 6. 加速度分析
        ax6 = fig.add_subplot(2, 3, 6)
        acceleration_cols = [col for col in traj_data.columns if col.startswith('acceleration_')]
        
        for i in range(min(3, len(position_cols))):
            if i < len(acceleration_cols):
                joint_name = position_cols[i].split('(')[-1].rstrip(')') if '(' in position_cols[i] else f"J{i}"
                ax6.plot(traj_data['time_from_start'], 
                        traj_data[acceleration_cols[i]], 
                        label=joint_name, linewidth=1.5, alpha=0.8)
        
        ax6.set_xlabel('时间 (s)')
        ax6.set_ylabel('加速度 (rad/s²)')
        ax6.set_title('关节加速度曲线（前3个关节）')
        ax6.grid(True, alpha=0.3)
        ax6.legend(loc='best', fontsize=8)
        
        plt.suptitle('PILZ焊接路径分析', fontsize=16, fontweight='bold')
        plt.tight_layout()
        
        output_file = 'pilz_welding_analysis.png'
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"\n✓ 图表已保存: {output_file}")
        plt.show()
    
    def plot_linearity_check(self):
        """
        绘制直线性检查图表
        """
        if self.waypoints_data is None:
            print("✗ 没有可用的路径点数据")
            return
        
        fig = plt.figure(figsize=(15, 10))
        
        data = self.waypoints_data
        start = data.iloc[0]
        end = data.iloc[-1]
        
        # 计算理想直线
        t = np.linspace(0, 1, len(data))
        ideal_x = start['x'] + t * (end['x'] - start['x'])
        ideal_y = start['y'] + t * (end['y'] - start['y'])
        ideal_z = start['z'] + t * (end['z'] - start['z'])
        
        # 计算偏差
        deviation_x = data['x'].values - ideal_x
        deviation_y = data['y'].values - ideal_y
        deviation_z = data['z'].values - ideal_z
        total_deviation = np.sqrt(deviation_x**2 + deviation_y**2 + deviation_z**2)
        
        # 1. 3D轨迹对比
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        ax1.plot(ideal_x, ideal_y, ideal_z, 'r--', linewidth=2, label='理想直线')
        ax1.plot(data['x'], data['y'], data['z'], 'b-', linewidth=2, label='实际路径')
        ax1.scatter(data['x'].iloc[0], data['y'].iloc[0], data['z'].iloc[0], 
                   c='green', s=150, marker='o')
        ax1.scatter(data['x'].iloc[-1], data['y'].iloc[-1], data['z'].iloc[-1], 
                   c='red', s=150, marker='s')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D轨迹直线性检查')
        ax1.legend()
        ax1.grid(True)
        
        # 2. XY投影对比
        ax2 = fig.add_subplot(2, 3, 2)
        ax2.plot(ideal_x, ideal_y, 'r--', linewidth=2, label='理想直线')
        ax2.plot(data['x'], data['y'], 'b-', linewidth=2, label='实际路径')
        ax2.scatter(data['x'].iloc[0], data['y'].iloc[0], c='green', s=100, marker='o')
        ax2.scatter(data['x'].iloc[-1], data['y'].iloc[-1], c='red', s=100, marker='s')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title('XY平面直线性检查')
        ax2.legend()
        ax2.grid(True)
        ax2.axis('equal')
        
        # 3. XZ投影对比
        ax3 = fig.add_subplot(2, 3, 3)
        ax3.plot(ideal_x, ideal_z, 'r--', linewidth=2, label='理想直线')
        ax3.plot(data['x'], data['z'], 'b-', linewidth=2, label='实际路径')
        ax3.scatter(data['x'].iloc[0], data['z'].iloc[0], c='green', s=100, marker='o')
        ax3.scatter(data['x'].iloc[-1], data['z'].iloc[-1], c='red', s=100, marker='s')
        ax3.set_xlabel('X (m)')
        ax3.set_ylabel('Z (m)')
        ax3.set_title('XZ平面直线性检查')
        ax3.legend()
        ax3.grid(True)
        
        # 4. 偏差分布
        ax4 = fig.add_subplot(2, 3, 4)
        ax4.plot(range(len(total_deviation)), total_deviation * 1000, 'b-', linewidth=1.5, alpha=0.7)
        ax4.fill_between(range(len(total_deviation)), total_deviation * 1000, alpha=0.3)
        ax4.axhline(y=0, color='r', linestyle='--', alpha=0.5)
        ax4.set_xlabel('路径点索引')
        ax4.set_ylabel('偏差 (mm)')
        ax4.set_title('总体偏差')
        ax4.grid(True, alpha=0.3)
        
        # 5. 各轴偏差
        ax5 = fig.add_subplot(2, 3, 5)
        ax5.plot(range(len(deviation_x)), deviation_x * 1000, label='ΔX', linewidth=1.5, alpha=0.8)
        ax5.plot(range(len(deviation_y)), deviation_y * 1000, label='ΔY', linewidth=1.5, alpha=0.8)
        ax5.plot(range(len(deviation_z)), deviation_z * 1000, label='ΔZ', linewidth=1.5, alpha=0.8)
        ax5.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        ax5.set_xlabel('路径点索引')
        ax5.set_ylabel('偏差 (mm)')
        ax5.set_title('各轴偏差对比')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # 6. 偏差直方图
        ax6 = fig.add_subplot(2, 3, 6)
        ax6.hist(total_deviation * 1000, bins=30, alpha=0.7, edgecolor='black')
        ax6.axvline(x=total_deviation.mean() * 1000, color='r', linestyle='--', 
                   label=f'平均: {total_deviation.mean()*1000:.3f}mm')
        ax6.axvline(x=total_deviation.max() * 1000, color='orange', linestyle=':', 
                   label=f'最大: {total_deviation.max()*1000:.3f}mm')
        ax6.set_xlabel('偏差 (mm)')
        ax6.set_ylabel('频数')
        ax6.set_title('偏差分布统计')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        
        plt.suptitle('PILZ焊接路径直线性检查', fontsize=16, fontweight='bold')
        plt.tight_layout()
        
        output_file = 'pilz_linearity_check.png'
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✓ 图表已保存: {output_file}")
        plt.show()
        
        # 打印统计信息
        print("\n" + "="*60)
        print("直线性统计")
        print("="*60)
        print(f"最大偏差: {total_deviation.max()*1000:.3f} mm")
        print(f"平均偏差: {total_deviation.mean()*1000:.3f} mm")
        print(f"标准差: {total_deviation.std()*1000:.3f} mm")
        print(f"中位数: {np.median(total_deviation)*1000:.3f} mm")
    
    def generate_report(self):
        """
        生成PILZ焊接路径分析报告
        """
        report = []
        report.append("="*70)
        report.append("PILZ焊接路径分析报告")
        report.append("="*70)
        report.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append("")
        
        # 焊接路径点信息
        if self.waypoints_data is not None:
            report.append("【焊接路径点信息】")
            report.append("-"*70)
            data = self.waypoints_data
            report.append(f"路径点数量: {len(data)}")
            report.append(f"总路径长度: {data['cumulative_distance'].iloc[-1]:.6f} m")
            
            if len(data) >= 2:
                start = data.iloc[0]
                end = data.iloc[-1]
                straight_dist = np.sqrt((end['x']-start['x'])**2 + (end['y']-start['y'])**2 + (end['z']-start['z'])**2)
                path_len = data['cumulative_distance'].iloc[-1]
                linearity = straight_dist / path_len if path_len > 0 else 0
                
                report.append(f"\n起点: [{start['x']:.6f}, {start['y']:.6f}, {start['z']:.6f}]")
                report.append(f"终点: [{end['x']:.6f}, {end['y']:.6f}, {end['z']:.6f}]")
                report.append(f"直线距离: {straight_dist:.6f} m")
                report.append(f"直线性系数: {linearity:.4f}")
                
                if linearity > 0.99:
                    report.append("结论: ✓ 路径是直线")
                elif linearity > 0.95:
                    report.append("结论: ⚠ 路径接近直线，存在轻微弯曲")
                else:
                    report.append("结论: ✗ 路径有明显弯曲")
            
            report.append("")
        
        # PILZ轨迹信息
        if self.trajectory_data is not None:
            report.append("【PILZ LIN轨迹信息】")
            report.append("-"*70)
            data = self.trajectory_data
            report.append(f"轨迹点数: {len(data)}")
            report.append(f"执行时间: {data['time_from_start'].iloc[-1]:.3f} 秒")
            
            time_steps = np.diff(data['time_from_start'])
            report.append(f"\n时间步长统计:")
            report.append(f"  平均值: {time_steps.mean():.6f} 秒")
            report.append(f"  最小值: {time_steps.min():.6f} 秒")
            report.append(f"  最大值: {time_steps.max():.6f} 秒")
            
            position_cols = [col for col in data.columns if col.startswith('position_')]
            velocity_cols = [col for col in data.columns if col.startswith('velocity_')]
            
            report.append(f"\n关节运动学特性 (前3个关节):")
            for i in range(min(3, len(position_cols))):
                joint_name = position_cols[i].split('(')[-1].rstrip(')') if '(' in position_cols[i] else f"J{i}"
                report.append(f"\n  {joint_name}:")
                
                if i < len(velocity_cols) and velocity_cols[i] in data.columns:
                    velocities = data[velocity_cols[i]].abs()
                    report.append(f"    最大速度: {velocities.max():.4f} rad/s")
                    report.append(f"    平均速度: {velocities.mean():.4f} rad/s")
            
            report.append("")
        
        # 直线性分析（如果有路径点数据）
        if self.waypoints_data is not None:
            data = self.waypoints_data
            start = data.iloc[0]
            end = data.iloc[-1]
            
            t = np.linspace(0, 1, len(data))
            ideal_x = start['x'] + t * (end['x'] - start['x'])
            ideal_y = start['y'] + t * (end['y'] - start['y'])
            ideal_z = start['z'] + t * (end['z'] - start['z'])
            
            deviation = np.sqrt((data['x'].values - ideal_x)**2 + 
                               (data['y'].values - ideal_y)**2 + 
                               (data['z'].values - ideal_z)**2)
            
            report.append("【直线性分析】")
            report.append("-"*70)
            report.append(f"最大偏差: {deviation.max()*1000:.3f} mm")
            report.append(f"平均偏差: {deviation.mean()*1000:.3f} mm")
            report.append(f"标准差: {deviation.std()*1000:.3f} mm")
            report.append(f"中位数: {np.median(deviation)*1000:.3f} mm")
            report.append("")
        
        # 生成报告
        report_text = '\n'.join(report)
        
        with open('pilz_welding_analysis_report.txt', 'w', encoding='utf-8') as f:
            f.write(report_text)
        
        print(report_text)
        print(f"\n✓ 报告已保存: pilz_welding_analysis_report.txt")


def main():
    """主函数"""
    if len(sys.argv) > 1:
        data_dir = sys.argv[1]
    else:
        data_dir = "."
    
    print("\n" + "="*70)
    print("PILZ焊接路径分析工具")
    print("="*70)
    
    # 创建分析器
    analyzer = PilzWeldingAnalyzer(data_dir)
    
    # 加载数据
    analyzer.load_pilz_data()
    
    # 执行分析
    analyzer.analyze_welding_waypoints()
    analyzer.analyze_pilz_trajectory()
    analyzer.compare_with_cartesian()
    
    # 生成图表
    print("\n生成图表中...")
    analyzer.plot_pilz_welding_analysis()
    analyzer.plot_linearity_check()
    
    # 生成报告
    analyzer.generate_report()
    
    print("\n" + "="*70)
    print("分析完成！")
    print("生成的文件:")
    print("  ✓ pilz_welding_analysis.png - PILZ焊接路径分析图表")
    print("  ✓ pilz_linearity_check.png - 直线性检查图表")
    print("  ✓ pilz_welding_analysis_report.txt - 文本报告")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
