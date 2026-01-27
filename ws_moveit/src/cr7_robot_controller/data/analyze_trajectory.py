#!/usr/bin/env python3
"""
轨迹分析工具
用于分析CR7机器人控制器的轨迹数据
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


# ================================
# 中文字体安全加载（完整修改版）
# ================================
def set_chinese_font():
    """
    安全设置中文字体
    优先使用常见字体 SimHei / Microsoft YaHei / WenQuanYi 等
    """
    # Linux 常见中文字体路径
    candidate_fonts = [
        "/usr/share/fonts/truetype/arphic/ukai.ttc",    # 文泉驿黑体
        "/usr/share/fonts/truetype/arphic/uming.ttc",   # 文泉驿正黑
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc", # Noto CJK
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc", # wqy-zenhei
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/fonts-japanese-gothic.ttf", # 日文字体
        "/usr/share/fonts/truetype/msttcorefonts/Arial.ttf",   # 英文可选
    ]

    font_path = None
    for f in candidate_fonts:
        if os.path.exists(f):
            font_path = f
            break

    if font_path:
        font_prop = font_manager.FontProperties(fname=font_path)
        plt.rcParams['font.family'] = font_prop.get_name()
        print(f"已设置中文字体: {font_prop.get_name()} ({font_path})")
    else:
        print("警告: 未找到可用中文字体，图表可能无法显示中文")

    plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# 设置中文字体
set_chinese_font()

def set_chinese_font():
    """
    安全设置中文字体
    优先使用常见字体 SimHei / Microsoft YaHei / WenQuanYi 等
    """
    import os
    from matplotlib import font_manager

    # Linux 常见中文字体路径
    candidate_fonts = [
        "/usr/share/fonts/truetype/arphic/ukai.ttc",    # 文泉驿黑体
        "/usr/share/fonts/truetype/arphic/uming.ttc",   # 文泉驿正黑
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc", # Noto CJK
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc", # wqy-zenhei
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/fonts-japanese-gothic.ttf", # 日文字体也可以
        "/usr/share/fonts/truetype/msttcorefonts/Arial.ttf",   # 英文可选
    ]

    font_path = None
    for f in candidate_fonts:
        if os.path.exists(f):
            font_path = f
            break

    if font_path:
        font_prop = font_manager.FontProperties(fname=font_path)
        plt.rcParams['font.family'] = font_prop.get_name()
        print(f"已设置中文字体: {font_prop.get_name()} ({font_path})")
    else:
        print("警告: 未找到可用中文字体，图表可能无法显示中文")
    
    plt.rcParams['axes.unicode_minus'] = False



class TrajectoryAnalyzer:
    def __init__(self, data_dir="."):
        """
        初始化分析器
        
        Args:
            data_dir: 数据文件目录
        """
        self.data_dir = data_dir
        self.waypoints_data = None
        self.trajectory_data = None
        self.optimized_data = None
        
    def load_latest_data(self):
        """
        加载最新的数据文件
        """
        # 查找最新的预处理路径点文件
        waypoint_files = glob.glob(os.path.join(self.data_dir, "preprocessed_waypoints_*.csv"))
        if waypoint_files:
            latest_waypoint = max(waypoint_files, key=os.path.getctime)
            self.waypoints_data = pd.read_csv(latest_waypoint)
            print(f"已加载预处理路径点文件: {latest_waypoint}")
            print(f"数据形状: {self.waypoints_data.shape}")
        else:
            print("未找到预处理路径点文件")
            self.waypoints_data = None
        
        # 查找原始轨迹文件
        original_files = glob.glob(os.path.join(self.data_dir, "original_trajectory_*.csv"))
        if original_files:
            latest_original = max(original_files, key=os.path.getctime)
            self.trajectory_data = pd.read_csv(latest_original)
            print(f"已加载原始轨迹文件: {latest_original}")
            print(f"数据形状: {self.trajectory_data.shape}")
        else:
            print("未找到原始轨迹文件")
            self.trajectory_data = None
        
        # 查找优化轨迹文件
        optimized_files = glob.glob(os.path.join(self.data_dir, "optimized_trajectory_*.csv"))
        if optimized_files:
            latest_optimized = max(optimized_files, key=os.path.getctime)
            self.optimized_data = pd.read_csv(latest_optimized)
            print(f"已加载优化轨迹文件: {latest_optimized}")
            print(f"数据形状: {self.optimized_data.shape}")
        else:
            print("未找到优化轨迹文件")
            self.optimized_data = None
    
    def analyze_waypoints(self):
        """
        分析路径点数据
        """
        if self.waypoints_data is None:
            print("没有可用的路径点数据")
            return
        
        print("\n" + "="*60)
        print("路径点数据分析")
        print("="*60)
        
        data = self.waypoints_data
        
        # 基本统计
        print(f"总路径点数: {len(data)}")
        print(f"总路径长度: {data['cumulative_distance'].iloc[-1]:.4f} m")
        
        # 计算距离统计
        distances = data['distance_from_prev'][1:]  # 跳过第一个点（距离为0）
        print(f"平均点间距: {distances.mean():.4f} m")
        print(f"最大点间距: {distances.max():.4f} m")
        print(f"最小点间距: {distances.min():.4f} m")
        print(f"点间距标准差: {distances.std():.4f} m")
        
        # 检查异常点
        threshold = 0.5  # 0.5m
        large_distances = distances[distances > threshold]
        if len(large_distances) > 0:
            print(f"\n警告: 发现 {len(large_distances)} 个间距大于 {threshold}m 的路径段")
            for idx, dist in large_distances.items():
                point_idx = idx + 1  # 因为跳过了第一个点
                print(f"  点 {point_idx-1} -> 点 {point_idx}: {dist:.4f} m")
        
        # 位置统计
        print(f"\nX坐标范围: [{data['x'].min():.4f}, {data['x'].max():.4f}]")
        print(f"Y坐标范围: [{data['y'].min():.4f}, {data['y'].max():.4f}]")
        print(f"Z坐标范围: [{data['z'].min():.4f}, {data['z'].max():.4f}]")
    
    def analyze_trajectory(self, trajectory_type="original"):
        """
        分析轨迹数据
        
        Args:
            trajectory_type: 'original' 或 'optimized'
        """
        if trajectory_type == "original":
            data = self.trajectory_data
            title = "原始轨迹"
        else:
            data = self.optimized_data
            title = "优化轨迹"
        
        if data is None:
            print(f"没有可用的{title}数据")
            return
        
        print(f"\n" + "="*60)
        print(f"{title}数据分析")
        print("="*60)
        
        print(f"轨迹点数: {len(data)}")
        print(f"轨迹总时间: {data['time_from_start'].iloc[-1]:.3f} 秒")
        
        # 查找关节列
        position_cols = [col for col in data.columns if col.startswith('position_')]
        velocity_cols = [col for col in data.columns if col.startswith('velocity_')]
        acceleration_cols = [col for col in data.columns if col.startswith('acceleration_')]
        
        print(f"关节数量: {len(position_cols)}")
        
        # 计算平均时间步长
        time_steps = np.diff(data['time_from_start'])
        print(f"平均时间步长: {time_steps.mean():.6f} 秒")
        print(f"最大时间步长: {time_steps.max():.6f} 秒")
        print(f"最小时间步长: {time_steps.min():.6f} 秒")
        
        # 分析每个关节
        for i, pos_col in enumerate(position_cols):
            joint_name = pos_col.split('(')[-1].rstrip(')') if '(' in pos_col else f"joint_{i}"
            
            # 位置分析
            positions = data[pos_col]
            pos_range = positions.max() - positions.min()
            pos_mean = positions.mean()
            
            # 速度分析
            vel_col = velocity_cols[i] if i < len(velocity_cols) else None
            if vel_col and vel_col in data.columns:
                velocities = data[vel_col]
                vel_max = velocities.abs().max()
                vel_mean = velocities.abs().mean()
            else:
                velocities = None
                vel_max = 0
                vel_mean = 0
            
            # 加速度分析
            acc_col = acceleration_cols[i] if i < len(acceleration_cols) else None
            if acc_col and acc_col in data.columns:
                accelerations = data[acc_col]
                acc_max = accelerations.abs().max()
                acc_mean = accelerations.abs().mean()
            else:
                accelerations = None
                acc_max = 0
                acc_mean = 0
            
            print(f"\n关节 {i} ({joint_name}):")
            print(f"  位置范围: [{positions.min():.4f}, {positions.max():.4f}] rad")
            print(f"  位置变化范围: {pos_range:.4f} rad")
            print(f"  平均位置: {pos_mean:.4f} rad")
            
            if velocities is not None:
                print(f"  最大速度: {vel_max:.4f} rad/s")
                print(f"  平均速度: {vel_mean:.4f} rad/s")
            
            if accelerations is not None:
                print(f"  最大加速度: {acc_max:.4f} rad/s²")
                print(f"  平均加速度: {acc_mean:.4f} rad/s²")
    
    def plot_3d_trajectory(self):
        """
        绘制3D轨迹图
        """
        if self.waypoints_data is None:
            print("没有可用的路径点数据")
            return
        
        fig = plt.figure(figsize=(15, 10))
        
        # 1. 3D轨迹图
        ax1 = fig.add_subplot(231, projection='3d')
        
        # 绘制路径点
        data = self.waypoints_data
        ax1.plot(data['x'], data['y'], data['z'], 'b-', linewidth=1, alpha=0.7, label='路径')
        ax1.scatter(data['x'], data['y'], data['z'], c='r', s=20, label='路径点')
        
        # 标记起点和终点
        ax1.scatter(data['x'].iloc[0], data['y'].iloc[0], data['z'].iloc[0], 
                   c='g', s=100, marker='o', label='起点')
        ax1.scatter(data['x'].iloc[-1], data['y'].iloc[-1], data['z'].iloc[-1], 
                   c='r', s=100, marker='s', label='终点')
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D轨迹路径')
        ax1.legend()
        ax1.grid(True)
        
        # 2. XY平面投影
        ax2 = fig.add_subplot(232)
        ax2.plot(data['x'], data['y'], 'b-', linewidth=1, alpha=0.7)
        ax2.scatter(data['x'], data['y'], c='r', s=20)
        ax2.scatter(data['x'].iloc[0], data['y'].iloc[0], c='g', s=100, marker='o')
        ax2.scatter(data['x'].iloc[-1], data['y'].iloc[-1], c='r', s=100, marker='s')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title('XY平面投影')
        ax2.grid(True)
        ax2.axis('equal')
        
        # 3. XZ平面投影
        ax3 = fig.add_subplot(233)
        ax3.plot(data['x'], data['z'], 'b-', linewidth=1, alpha=0.7)
        ax3.scatter(data['x'], data['z'], c='r', s=20)
        ax3.scatter(data['x'].iloc[0], data['z'].iloc[0], c='g', s=100, marker='o')
        ax3.scatter(data['x'].iloc[-1], data['z'].iloc[-1], c='r', s=100, marker='s')
        ax3.set_xlabel('X (m)')
        ax3.set_ylabel('Z (m)')
        ax3.set_title('XZ平面投影')
        ax3.grid(True)
        
        # 4. 距离分布
        ax4 = fig.add_subplot(234)
        distances = data['distance_from_prev'][1:]  # 跳过第一个点
        ax4.bar(range(len(distances)), distances, alpha=0.7)
        ax4.axhline(y=distances.mean(), color='r', linestyle='--', label=f'平均值: {distances.mean():.4f}m')
        ax4.axhline(y=0.15, color='g', linestyle=':', label='阈值: 0.15m')
        ax4.set_xlabel('路径段索引')
        ax4.set_ylabel('距离 (m)')
        ax4.set_title('路径点间距分布')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # 5. 累积距离
        ax5 = fig.add_subplot(235)
        ax5.plot(data.index, data['cumulative_distance'], 'b-', linewidth=2)
        ax5.set_xlabel('路径点索引')
        ax5.set_ylabel('累积距离 (m)')
        ax5.set_title('累积路径长度')
        ax5.grid(True)
        
        # 6. 坐标变化
        ax6 = fig.add_subplot(236)
        ax6.plot(data.index, data['x'], 'r-', label='X', linewidth=2)
        ax6.plot(data.index, data['y'], 'g-', label='Y', linewidth=2)
        ax6.plot(data.index, data['z'], 'b-', label='Z', linewidth=2)
        ax6.set_xlabel('路径点索引')
        ax6.set_ylabel('坐标值 (m)')
        ax6.set_title('坐标变化')
        ax6.legend()
        ax6.grid(True)
        
        plt.suptitle('路径点分析图表', fontsize=16, fontweight='bold')
        plt.tight_layout()
        plt.savefig('waypoints_analysis.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_trajectory_comparison(self):
        """
        绘制轨迹对比图
        """
        if self.trajectory_data is None:
            print("没有可用的轨迹数据")
            return
        
        fig = plt.figure(figsize=(18, 12))
        
        # 查找关节列
        position_cols = [col for col in self.trajectory_data.columns if col.startswith('position_')]
        n_joints = len(position_cols)
        
        if n_joints == 0:
            print("没有找到关节位置数据")
            return
        
        # 1. 关节位置对比
        ax1 = fig.add_subplot(3, 2, 1)
        for i, pos_col in enumerate(position_cols):
            joint_name = pos_col.split('(')[-1].rstrip(')') if '(' in pos_col else f"关节{i}"
            ax1.plot(self.trajectory_data['time_from_start'], 
                    self.trajectory_data[pos_col], 
                    label=joint_name, linewidth=2, alpha=0.8)
        
        if self.optimized_data is not None:
            for i, pos_col in enumerate(position_cols):
                if i < len([col for col in self.optimized_data.columns if col.startswith('position_')]):
                    ax1.plot(self.optimized_data['time_from_start'], 
                            self.optimized_data[pos_col], 
                            '--', linewidth=1, alpha=0.5)
        
        ax1.set_xlabel('时间 (s)')
        ax1.set_ylabel('关节位置 (rad)')
        ax1.set_title('关节位置随时间变化')
        ax1.legend(loc='best')
        ax1.grid(True, alpha=0.3)
        
        # 2. 关节速度对比
        ax2 = fig.add_subplot(3, 2, 2)
        velocity_cols = [col for col in self.trajectory_data.columns if col.startswith('velocity_')]
        
        for i, vel_col in enumerate(velocity_cols):
            if i >= len(position_cols):
                break
            joint_name = position_cols[i].split('(')[-1].rstrip(')') if '(' in position_cols[i] else f"关节{i}"
            ax2.plot(self.trajectory_data['time_from_start'], 
                    self.trajectory_data[vel_col], 
                    label=joint_name, linewidth=2, alpha=0.8)
        
        ax2.set_xlabel('时间 (s)')
        ax2.set_ylabel('关节速度 (rad/s)')
        ax2.set_title('关节速度随时间变化')
        ax2.legend(loc='best')
        ax2.grid(True, alpha=0.3)
        
        # 3. 位置变化率
        ax3 = fig.add_subplot(3, 2, 3)
        time_data = self.trajectory_data['time_from_start'].values
        colors = plt.cm.rainbow(np.linspace(0, 1, n_joints))
        
        for i, pos_col in enumerate(position_cols):
            joint_name = pos_col.split('(')[-1].rstrip(')') if '(' in pos_col else f"关节{i}"
            positions = self.trajectory_data[pos_col].values
            
            # 计算位置变化
            position_changes = np.abs(np.diff(positions))
            ax3.plot(time_data[1:], position_changes, 
                    color=colors[i], label=joint_name, linewidth=2, alpha=0.7)
        
        ax3.set_xlabel('时间 (s)')
        ax3.set_ylabel('位置变化 (rad)')
        ax3.set_title('关节位置变化率')
        ax3.grid(True, alpha=0.3)
        
        # 4. 轨迹点密度
        ax4 = fig.add_subplot(3, 2, 4)
        time_intervals = np.diff(time_data)
        ax4.plot(time_data[1:], time_intervals, 'b-', linewidth=2, alpha=0.7)
        ax4.axhline(y=np.mean(time_intervals), color='r', linestyle='--', 
                   label=f'平均间隔: {np.mean(time_intervals):.4f}s')
        ax4.set_xlabel('时间 (s)')
        ax4.set_ylabel('时间间隔 (s)')
        ax4.set_title('轨迹点时间间隔')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # 5. 相位图（位置 vs 速度）
        ax5 = fig.add_subplot(3, 2, 5)
        for i, (pos_col, vel_col) in enumerate(zip(position_cols[:3], velocity_cols[:3])):
            if vel_col in self.trajectory_data.columns:
                joint_name = pos_col.split('(')[-1].rstrip(')') if '(' in pos_col else f"关节{i}"
                ax5.plot(self.trajectory_data[pos_col], 
                        self.trajectory_data[vel_col], 
                        label=joint_name, linewidth=2, alpha=0.7)
        
        ax5.set_xlabel('关节位置 (rad)')
        ax5.set_ylabel('关节速度 (rad/s)')
        ax5.set_title('相位图 (位置 vs 速度)')
        ax5.legend(loc='best')
        ax5.grid(True, alpha=0.3)
        
        # 6. 轨迹优化对比
        ax6 = fig.add_subplot(3, 2, 6)
        if self.optimized_data is not None:
            original_points = len(self.trajectory_data)
            optimized_points = len(self.optimized_data)
            
            categories = ['原始轨迹', '优化轨迹']
            counts = [original_points, optimized_points]
            times = [self.trajectory_data['time_from_start'].iloc[-1], 
                    self.optimized_data['time_from_start'].iloc[-1]]
            
            x = np.arange(len(categories))
            width = 0.35
            
            ax6.bar(x - width/2, counts, width, label='轨迹点数', alpha=0.8)
            ax6.bar(x + width/2, times, width, label='总时间(s)', alpha=0.8)
            
            ax6.set_xlabel('轨迹类型')
            ax6.set_ylabel('数量/时间')
            ax6.set_title('轨迹优化对比')
            ax6.set_xticks(x)
            ax6.set_xticklabels(categories)
            ax6.legend()
            
            # 添加数值标签
            for i, v in enumerate(counts):
                ax6.text(i - width/2, v + 0.1, str(v), ha='center')
            for i, v in enumerate(times):
                ax6.text(i + width/2, v + 0.1, f"{v:.2f}", ha='center')
            
            reduction = (original_points - optimized_points) / original_points * 100
            ax6.text(0.5, max(max(counts), max(times)) * 0.5, 
                    f"点减少: {reduction:.1f}%", 
                    ha='center', fontsize=12, fontweight='bold',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.5))
        else:
            ax6.text(0.5, 0.5, '无优化轨迹数据', 
                    ha='center', va='center', fontsize=12)
            ax6.axis('off')
        
        plt.suptitle('轨迹数据分析图表', fontsize=16, fontweight='bold')
        plt.tight_layout()
        plt.savefig('trajectory_analysis.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_joint_specific_analysis(self, joint_index=0):
        """
        绘制特定关节的详细分析
        
        Args:
            joint_index: 关节索引
        """
        if self.trajectory_data is None:
            print("没有可用的轨迹数据")
            return
        
        # 查找指定关节的列
        position_cols = [col for col in self.trajectory_data.columns if col.startswith('position_')]
        velocity_cols = [col for col in self.trajectory_data.columns if col.startswith('velocity_')]
        acceleration_cols = [col for col in self.trajectory_data.columns if col.startswith('acceleration_')]
        
        if joint_index >= len(position_cols):
            print(f"关节索引 {joint_index} 超出范围 (最大: {len(position_cols)-1})")
            return
        
        pos_col = position_cols[joint_index]
        vel_col = velocity_cols[joint_index] if joint_index < len(velocity_cols) else None
        acc_col = acceleration_cols[joint_index] if joint_index < len(acceleration_cols) else None
        
        joint_name = pos_col.split('(')[-1].rstrip(')') if '(' in pos_col else f"关节{joint_index}"
        
        fig = plt.figure(figsize=(15, 10))
        
        # 1. 位置、速度、加速度
        time_data = self.trajectory_data['time_from_start']
        
        ax1 = fig.add_subplot(2, 2, 1)
        ax1.plot(time_data, self.trajectory_data[pos_col], 'b-', linewidth=2, label='位置')
        ax1.set_xlabel('时间 (s)')
        ax1.set_ylabel('位置 (rad)', color='b')
        ax1.tick_params(axis='y', labelcolor='b')
        ax1.set_title(f'{joint_name} - 位置')
        ax1.grid(True, alpha=0.3)
        
        if vel_col and vel_col in self.trajectory_data.columns:
            ax2 = ax1.twinx()
            ax2.plot(time_data, self.trajectory_data[vel_col], 'r-', linewidth=2, alpha=0.7, label='速度')
            ax2.set_ylabel('速度 (rad/s)', color='r')
            ax2.tick_params(axis='y', labelcolor='r')
            
            # 合并图例
            lines1, labels1 = ax1.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax1.legend(lines1 + lines2, labels1 + labels2, loc='best')
        
        # 2. 位置直方图
        ax3 = fig.add_subplot(2, 2, 2)
        positions = self.trajectory_data[pos_col]
        ax3.hist(positions, bins=30, alpha=0.7, edgecolor='black')
        ax3.axvline(positions.mean(), color='r', linestyle='--', 
                   label=f'平均值: {positions.mean():.4f}')
        ax3.axvline(positions.median(), color='g', linestyle=':', 
                   label=f'中位数: {positions.median():.4f}')
        ax3.set_xlabel('位置 (rad)')
        ax3.set_ylabel('频数')
        ax3.set_title(f'{joint_name} - 位置分布')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 3. 速度分析
        ax4 = fig.add_subplot(2, 2, 3)
        if vel_col and vel_col in self.trajectory_data.columns:
            velocities = self.trajectory_data[vel_col]
            ax4.plot(time_data, velocities, 'g-', linewidth=2)
            ax4.set_xlabel('时间 (s)')
            ax4.set_ylabel('速度 (rad/s)')
            ax4.set_title(f'{joint_name} - 速度')
            ax4.grid(True, alpha=0.3)
            
            # 添加统计信息
            stats_text = (f'最大速度: {velocities.abs().max():.4f} rad/s\n'
                         f'平均速度: {velocities.abs().mean():.4f} rad/s\n'
                         f'速度标准差: {velocities.std():.4f} rad/s')
            ax4.text(0.02, 0.98, stats_text, transform=ax4.transAxes,
                    verticalalignment='top', fontsize=10,
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        else:
            ax4.text(0.5, 0.5, '无速度数据', 
                    ha='center', va='center', fontsize=12)
            ax4.axis('off')
        
        # 4. 加速度分析
        ax5 = fig.add_subplot(2, 2, 4)
        if acc_col and acc_col in self.trajectory_data.columns:
            accelerations = self.trajectory_data[acc_col]
            ax5.plot(time_data, accelerations, 'purple', linewidth=2)
            ax5.set_xlabel('时间 (s)')
            ax5.set_ylabel('加速度 (rad/s²)')
            ax5.set_title(f'{joint_name} - 加速度')
            ax5.grid(True, alpha=0.3)
            
            # 添加统计信息
            stats_text = (f'最大加速度: {accelerations.abs().max():.4f} rad/s²\n'
                         f'平均加速度: {accelerations.abs().mean():.4f} rad/s²\n'
                         f'加速度标准差: {accelerations.std():.4f} rad/s²')
            ax5.text(0.02, 0.98, stats_text, transform=ax5.transAxes,
                    verticalalignment='top', fontsize=10,
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        else:
            ax5.text(0.5, 0.5, '无加速度数据', 
                    ha='center', va='center', fontsize=12)
            ax5.axis('off')
        
        plt.suptitle(f'关节 {joint_name} 详细分析', fontsize=16, fontweight='bold')
        plt.tight_layout()
        plt.savefig(f'joint_{joint_index}_analysis.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def generate_report(self):
        """
        生成分析报告
        """
        report = []
        report.append("="*60)
        report.append("CR7机器人轨迹分析报告")
        report.append("="*60)
        report.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append("")
        
        # 路径点分析
        if self.waypoints_data is not None:
            report.append("路径点分析:")
            report.append("-"*40)
            data = self.waypoints_data
            report.append(f"总路径点数: {len(data)}")
            report.append(f"总路径长度: {data['cumulative_distance'].iloc[-1]:.4f} m")
            
            distances = data['distance_from_prev'][1:]
            report.append(f"平均点间距: {distances.mean():.4f} m")
            report.append(f"最大点间距: {distances.max():.4f} m")
            report.append(f"最小点间距: {distances.min():.4f} m")
            report.append(f"点间距标准差: {distances.std():.4f} m")
            report.append("")
        
        # 轨迹分析
        if self.trajectory_data is not None:
            report.append("原始轨迹分析:")
            report.append("-"*40)
            data = self.trajectory_data
            report.append(f"轨迹点数: {len(data)}")
            report.append(f"轨迹总时间: {data['time_from_start'].iloc[-1]:.3f} 秒")
            
            time_steps = np.diff(data['time_from_start'])
            report.append(f"平均时间步长: {time_steps.mean():.6f} 秒")
            report.append(f"最大时间步长: {time_steps.max():.6f} 秒")
            report.append(f"最小时间步长: {time_steps.min():.6f} 秒")
            report.append("")
        
        # 优化轨迹对比
        if self.optimized_data is not None:
            report.append("优化轨迹对比:")
            report.append("-"*40)
            original_len = len(self.trajectory_data)
            optimized_len = len(self.optimized_data)
            reduction = (original_len - optimized_len) / original_len * 100
            
            report.append(f"原始轨迹点数: {original_len}")
            report.append(f"优化轨迹点数: {optimized_len}")
            report.append(f"点减少比例: {reduction:.1f}%")
            report.append("")
        
        # 保存报告
        with open('trajectory_analysis_report.txt', 'w', encoding='utf-8') as f:
            f.write('\n'.join(report))
        
        print('\n'.join(report))
        print("\n报告已保存到: trajectory_analysis_report.txt")

def main():
    """
    主函数
    """
    # 检查命令行参数
    if len(sys.argv) > 1:
        data_dir = sys.argv[1]
    else:
        data_dir = "."
    
    # 创建分析器
    analyzer = TrajectoryAnalyzer(data_dir)
    
    # 加载数据
    analyzer.load_latest_data()
    
    # 生成分析
    analyzer.analyze_waypoints()
    analyzer.analyze_trajectory("original")
    
    if analyzer.optimized_data is not None:
        analyzer.analyze_trajectory("optimized")
    
    # 生成图表
    print("\n生成图表中...")
    
    if analyzer.waypoints_data is not None:
        analyzer.plot_3d_trajectory()
    
    if analyzer.trajectory_data is not None:
        analyzer.plot_trajectory_comparison()
        
        # 分析第一个关节
        if len([col for col in analyzer.trajectory_data.columns if col.startswith('position_')]) > 0:
            analyzer.plot_joint_specific_analysis(0)
    
    # 生成报告
    analyzer.generate_report()
    
    print("\n分析完成！")
    print("生成的图表:")
    print("  1. waypoints_analysis.png - 路径点分析图表")
    print("  2. trajectory_analysis.png - 轨迹分析图表")
    print("  3. joint_0_analysis.png - 关节0详细分析")
    print("  4. trajectory_analysis_report.txt - 文本报告")

if __name__ == "__main__":
    main()