#!/usr/bin/env python3
"""
轨迹数据日志分析工具
用于分析CR7机器人控制器的轨迹日志数据，验证数据正确性
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import glob
import os
import sys
from datetime import datetime
from matplotlib import font_manager


# ================================
# 中文字体安全加载
# ================================
def set_chinese_font():
    """
    安全设置中文字体，支持Windows和Linux系统
    """
    # Windows系统常见中文字体
    windows_fonts = [
        "C:\\Windows\\Fonts\\SimHei.ttf",  # 黑体
        "C:\\Windows\\Fonts\\Microsoft YaHei UI\\msyh.ttc",  # 微软雅黑
        "C:\\Windows\\Fonts\\Microsoft YaHei UI\\msyhbd.ttc",  # 微软雅黑粗体
        "C:\\Windows\\Fonts\\simsun.ttc",  # 宋体
        "C:\\Windows\\Fonts\\simkai.ttf",  # 楷体
    ]
    
    # Linux系统常见中文字体
    linux_fonts = [
        "/usr/share/fonts/truetype/arphic/ukai.ttc",
        "/usr/share/fonts/truetype/arphic/uming.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    ]

    # 先尝试Windows字体
    font_path = None
    for f in windows_fonts:
        if os.path.exists(f):
            font_path = f
            break
    
    # 如果Windows字体不存在，尝试Linux字体
    if not font_path:
        for f in linux_fonts:
            if os.path.exists(f):
                font_path = f
                break

    if font_path:
        font_prop = font_manager.FontProperties(fname=font_path)
        plt.rcParams['font.family'] = font_prop.get_name()
        print(f"✓ 已设置中文字体: {font_prop.get_name()} ({font_path})")
    else:
        # 如果没有找到中文字体，尝试使用系统默认字体
        plt.rcParams['font.family'] = ['SimHei', 'WenQuanYi Micro Hei', 'Heiti TC', 'sans-serif']
        print("⚠ 警告: 未找到可用中文字体，使用默认字体设置")

    plt.rcParams['axes.unicode_minus'] = False


set_chinese_font()


class LogDataAnalyzer:
    """轨迹日志数据分析器"""
    
    def __init__(self, data_dir="."):
        """
        初始化分析器
        
        Args:
            data_dir: 数据文件目录
        """
        self.data_dir = data_dir
        self.log_files = []
        self.parsed_data = {}
        
    def find_log_files(self):
        """
        查找所有轨迹日志文件
        """
        print("\n" + "="*60)
        print("查找轨迹日志文件")
        print("="*60)
        
        # 查找所有轨迹文件
        self.log_files = glob.glob(os.path.join(self.data_dir, "*.txt"))
        
        if self.log_files:
            print(f"✓ 找到 {len(self.log_files)} 个轨迹日志文件")
            for file in self.log_files:
                print(f"  - {os.path.basename(file)}")
        else:
            print("✗ 未找到轨迹日志文件")
    
    def parse_log_file(self, file_path):
        """
        解析单个日志文件
        
        Args:
            file_path: 文件路径
        
        Returns:
            解析后的数据字典
        """
        print(f"\n解析文件: {os.path.basename(file_path)}")
        
        data = {
            'file_name': os.path.basename(file_path),
            'trajectory_points': [],
            'total_points': 0,
            'total_time': 0.0,
            'joint_names': []
        }
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            # 解析文件头部信息
            for i, line in enumerate(lines):
                line = line.strip()
                
                if "轨迹点数:" in line:
                    data['total_points'] = int(line.split(":")[1].strip())
                elif "轨迹总时间:" in line:
                    data['total_time'] = float(line.split(":")[1].strip().split()[0])
                elif "关节名称:" in line:
                    # 读取关节名称
                    for j in range(i+1, len(lines)):
                        joint_line = lines[j].strip()
                        if joint_line and ":" in joint_line:
                            parts = joint_line.split(":")
                            if len(parts) == 2:
                                joint_name = parts[1].strip()
                                data['joint_names'].append(joint_name)
                        else:
                            break
                elif "详细轨迹点信息:" in line:
                    # 开始解析轨迹点
                    trajectory_start = i + 1
                    break
            
            # 解析轨迹点
            trajectory_points = []
            current_point = None
            
            for line in lines[trajectory_start:]:
                line = line.strip()
                
                if line.startswith("轨迹点"):
                    # 新的轨迹点
                    if current_point:
                        trajectory_points.append(current_point)
                    
                    point_idx = int(line.split()[1].rstrip(':'))
                    current_point = {
                        'index': point_idx,
                        'time': 0.0,
                        'positions': [],
                        'velocities': [],
                        'accelerations': [],
                        'cartesian_position': [],
                        'cartesian_orientation': []
                    }
                elif "时间:" in line:
                    current_point['time'] = float(line.split(":")[1].strip().split()[0])
                elif "关节位置:" in line:
                    pos_str = line.split(":")[1].strip()
                    positions = list(map(float, pos_str.strip('[]').split(', ')))
                    current_point['positions'] = positions
                elif "关节速度:" in line:
                    vel_str = line.split(":")[1].strip()
                    velocities = list(map(float, vel_str.strip('[]').split(', ')))
                    current_point['velocities'] = velocities
                elif "关节加速度:" in line:
                    acc_str = line.split(":")[1].strip()
                    accelerations = list(map(float, acc_str.strip('[]').split(', ')))
                    current_point['accelerations'] = accelerations
                elif "笛卡尔位置:" in line:
                    cart_pos_str = line.split(":")[1].strip()
                    if cart_pos_str != "无法计算":
                        cart_positions = list(map(float, cart_pos_str.strip('[]').split(', ')))
                        current_point['cartesian_position'] = cart_positions
                elif "笛卡尔姿态:" in line:
                    cart_orient_str = line.split(":")[1].strip()
                    if cart_orient_str != "无法计算":
                        cart_orientations = list(map(float, cart_orient_str.strip('[]').split(', ')))
                        current_point['cartesian_orientation'] = cart_orientations
            
            # 添加最后一个点
            if current_point:
                trajectory_points.append(current_point)
            
            data['trajectory_points'] = trajectory_points
            
            # 验证数据完整性
            if len(trajectory_points) == data['total_points']:
                print(f"✓ 成功解析 {len(trajectory_points)} 个轨迹点")
            else:
                print(f"⚠ 警告: 解析的轨迹点数 ({len(trajectory_points)}) 与文件声明的 ({data['total_points']}) 不匹配")
            
        except Exception as e:
            print(f"✗ 解析文件失败: {e}")
            return None
        
        return data
    
    def parse_all_files(self):
        """
        解析所有日志文件
        """
        for file_path in self.log_files:
            data = self.parse_log_file(file_path)
            if data:
                self.parsed_data[os.path.basename(file_path)] = data
        
        print(f"\n✓ 完成解析 {len(self.parsed_data)} 个文件")
    
    def analyze_data_correctness(self):
        """
        分析数据正确性
        """
        print("\n" + "="*60)
        print("数据正确性分析")
        print("="*60)
        
        for file_name, data in self.parsed_data.items():
            print(f"\n分析文件: {file_name}")
            print("-"*40)
            
            # 1. 基本信息验证
            print(f"轨迹点数: {data['total_points']}")
            print(f"轨迹总时间: {data['total_time']:.3f} 秒")
            print(f"关节数量: {len(data['joint_names'])}")
            
            # 2. 时间序列验证
            times = [point['time'] for point in data['trajectory_points']]
            time_diffs = np.diff(times)
            
            if len(time_diffs) > 0:
                print(f"\n时间序列分析:")
                print(f"  平均时间步长: {time_diffs.mean():.6f} 秒")
                print(f"  最大时间步长: {time_diffs.max():.6f} 秒")
                print(f"  最小时间步长: {time_diffs.min():.6f} 秒")
                
                # 检查时间是否单调递增
                if all(diff >= -1e-6 for diff in time_diffs):  # 允许小误差
                    print("  ✓ 时间序列单调递增")
                else:
                    print("  ✗ 时间序列不是单调递增")
            
            # 3. 关节数据验证
            print("\n关节数据分析:")
            for i, joint_name in enumerate(data['joint_names']):
                positions = []
                velocities = []
                accelerations = []
                
                for point in data['trajectory_points']:
                    # 确保数据完整性
                    if i < len(point['positions']):
                        positions.append(point['positions'][i])
                    if i < len(point['velocities']):
                        velocities.append(point['velocities'][i])
                    if i < len(point['accelerations']):
                        accelerations.append(point['accelerations'][i])
                
                print(f"\n  {joint_name}:")
                if positions:
                    print(f"    位置范围: [{min(positions):.4f}, {max(positions):.4f}] rad")
                if velocities:
                    print(f"    速度范围: [{min(velocities):.4f}, {max(velocities):.4f}] rad/s")
                if accelerations:
                    print(f"    加速度范围: [{min(accelerations):.4f}, {max(accelerations):.4f}] rad/s²")
                
                # 检查速度和加速度的合理性
                if velocities:
                    max_velocity = max(abs(v) for v in velocities)
                    if max_velocity > 5.0:  # 假设最大速度限制为5 rad/s
                        print(f"    ⚠ 警告: 最大速度 {max_velocity:.4f} 可能超出合理范围")
                if accelerations:
                    max_acceleration = max(abs(a) for a in accelerations)
                    if max_acceleration > 20.0:  # 假设最大加速度限制为20 rad/s²
                        print(f"    ⚠ 警告: 最大加速度 {max_acceleration:.4f} 可能超出合理范围")
            
            # 4. 轨迹完整性验证
            print("\n轨迹完整性验证:")
            if len(data['trajectory_points']) > 0:
                start_point = data['trajectory_points'][0]
                end_point = data['trajectory_points'][-1]
                
                print(f"  起点时间: {start_point['time']:.3f} 秒")
                print(f"  终点时间: {end_point['time']:.3f} 秒")
                print(f"  实际轨迹时间: {end_point['time'] - start_point['time']:.3f} 秒")
                
                if abs(end_point['time'] - data['total_time']) < 1e-3:
                    print("  ✓ 轨迹时间与声明一致")
                else:
                    print(f"  ⚠ 警告: 轨迹时间与声明不一致 (声明: {data['total_time']:.3f}, 实际: {end_point['time']:.3f})")
            
            print("-"*40)
    
    def plot_trajectory_analysis(self, output_dir):
        """
        绘制轨迹分析图表
        
        Args:
            output_dir: 输出文件夹路径
        """
        print("\n" + "="*60)
        print("生成轨迹分析图表")
        print("="*60)
        
        for file_name, data in self.parsed_data.items():
            print(f"\n生成 {file_name} 的分析图表")
            
            # 创建图表
            fig = plt.figure(figsize=(20, 15))
            
            # 1. 时间序列分析
            ax1 = fig.add_subplot(3, 3, 1)
            times = [point['time'] for point in data['trajectory_points']]
            time_diffs = np.diff(times)
            ax1.plot(times[1:], time_diffs, 'b-', linewidth=1.5, alpha=0.7)
            ax1.axhline(y=np.mean(time_diffs), color='r', linestyle='--', 
                       label=f'平均: {np.mean(time_diffs):.4f}s')
            ax1.set_xlabel('时间 (s)')
            ax1.set_ylabel('时间间隔 (s)')
            ax1.set_title('时间间隔分析')
            ax1.grid(True, alpha=0.3)
            ax1.legend()
            
            # 2. 关节位置分析（全部6个关节）
            ax2 = fig.add_subplot(3, 3, 2)
            for i, joint_name in enumerate(data['joint_names']):  # 显示全部关节
                positions = []
                for point in data['trajectory_points']:
                    if i < len(point['positions']):
                        positions.append(point['positions'][i])
                if positions and len(positions) == len(times):
                    ax2.plot(times, positions, label=joint_name, linewidth=1.5, alpha=0.8)
            ax2.set_xlabel('时间 (s)')
            ax2.set_ylabel('关节位置 (rad)')
            ax2.set_title('关节位置曲线')
            ax2.grid(True, alpha=0.3)
            ax2.legend()
            
            # 3. 关节速度分析（全部6个关节）
            ax3 = fig.add_subplot(3, 3, 3)
            for i, joint_name in enumerate(data['joint_names']):
                velocities = []
                for point in data['trajectory_points']:
                    if i < len(point['velocities']):
                        velocities.append(point['velocities'][i])
                if velocities and len(velocities) == len(times):
                    ax3.plot(times, velocities, label=joint_name, linewidth=1.5, alpha=0.8)
            ax3.set_xlabel('时间 (s)')
            ax3.set_ylabel('关节速度 (rad/s)')
            ax3.set_title('关节速度曲线')
            ax3.grid(True, alpha=0.3)
            ax3.legend()
            
            # 4. 关节加速度分析（全部6个关节）
            ax4 = fig.add_subplot(3, 3, 4)
            for i, joint_name in enumerate(data['joint_names']):
                accelerations = []
                for point in data['trajectory_points']:
                    if i < len(point['accelerations']):
                        accelerations.append(point['accelerations'][i])
                if accelerations and len(accelerations) == len(times):
                    ax4.plot(times, accelerations, label=joint_name, linewidth=1.5, alpha=0.8)
            ax4.set_xlabel('时间 (s)')
            ax4.set_ylabel('关节加速度 (rad/s²)')
            ax4.set_title('关节加速度曲线')
            ax4.grid(True, alpha=0.3)
            ax4.legend()
            
            # 5. 笛卡尔位置轨迹（3D）
            ax5 = fig.add_subplot(3, 3, 5, projection='3d')
            cart_positions = []
            for point in data['trajectory_points']:
                if 'cartesian_position' in point and point['cartesian_position']:
                    cart_positions.append(point['cartesian_position'])
            if cart_positions:
                cart_positions = np.array(cart_positions)
                ax5.plot(cart_positions[:, 0], cart_positions[:, 1], cart_positions[:, 2], 
                         'b-', linewidth=1.5, alpha=0.8)
                ax5.scatter(cart_positions[0, 0], cart_positions[0, 1], cart_positions[0, 2], 
                           color='g', s=100, label='起点')
                ax5.scatter(cart_positions[-1, 0], cart_positions[-1, 1], cart_positions[-1, 2], 
                           color='r', s=100, label='终点')
                ax5.set_xlabel('X (m)')
                ax5.set_ylabel('Y (m)')
                ax5.set_zlabel('Z (m)')
                ax5.set_title('笛卡尔空间轨迹')
                ax5.grid(True, alpha=0.3)
                ax5.legend()
            else:
                ax5.text(0.5, 0.5, 0.5, '无笛卡尔数据', ha='center', va='center')
            
            # 6. 笛卡尔位置随时间变化
            ax6 = fig.add_subplot(3, 3, 6)
            if cart_positions.size > 0:
                ax6.plot(times, cart_positions[:, 0], label='X', linewidth=1.5, alpha=0.8)
                ax6.plot(times, cart_positions[:, 1], label='Y', linewidth=1.5, alpha=0.8)
                ax6.plot(times, cart_positions[:, 2], label='Z', linewidth=1.5, alpha=0.8)
                ax6.set_xlabel('时间 (s)')
                ax6.set_ylabel('笛卡尔位置 (m)')
                ax6.set_title('笛卡尔位置随时间变化')
                ax6.grid(True, alpha=0.3)
                ax6.legend()
            else:
                ax6.text(0.5, 0.5, '无笛卡尔数据', ha='center', va='center')
                ax6.axis('off')
            
            # 7. 速度分布直方图
            ax7 = fig.add_subplot(3, 3, 7)
            all_velocities = []
            for i in range(len(data['joint_names'])):
                for point in data['trajectory_points']:
                    if i < len(point['velocities']):
                        all_velocities.append(abs(point['velocities'][i]))
            if all_velocities:
                ax7.hist(all_velocities, bins=30, alpha=0.7, edgecolor='black')
                ax7.axvline(x=np.mean(all_velocities), color='r', linestyle='--', 
                           label=f'平均: {np.mean(all_velocities):.4f}')
                ax7.set_xlabel('速度绝对值 (rad/s)')
                ax7.set_ylabel('频数')
                ax7.set_title('速度分布')
                ax7.grid(True, alpha=0.3)
                ax7.legend()
            else:
                ax7.text(0.5, 0.5, '无速度数据', ha='center', va='center')
                ax7.axis('off')
            
            # 8. 轨迹点密度分析
            ax8 = fig.add_subplot(3, 3, 8)
            time_intervals = np.diff(times)
            ax8.plot(times[1:], 1.0 / time_intervals, 'b-', linewidth=1.5, alpha=0.7)
            ax8.set_xlabel('时间 (s)')
            ax8.set_ylabel('点密度 (1/s)')
            ax8.set_title('轨迹点密度')
            ax8.grid(True, alpha=0.3)
            
            # 9. 空白占位
            ax9 = fig.add_subplot(3, 3, 9)
            ax9.axis('off')
            
            plt.suptitle(f'{file_name} 轨迹分析', fontsize=16, fontweight='bold')
            plt.tight_layout()
            
            # 保存图表到输出文件夹
            output_file = os.path.join(output_dir, f"{os.path.splitext(file_name)[0]}_analysis.png")
            plt.savefig(output_file, dpi=300, bbox_inches='tight')
            print(f"✓ 图表已保存: {output_file}")
            plt.close()
    
    def generate_analysis_report(self, output_dir):
        """
        生成分析报告，为每个数据文件生成单独的报告
        
        Args:
            output_dir: 输出文件夹路径
        """
        print("\n" + "="*60)
        print("生成分析报告")
        print("="*60)
        
        # 为每个文件生成单独的报告
        for file_name, data in self.parsed_data.items():
            print(f"\n生成 {file_name} 的分析报告")
            
            report = []
            report.append("="*70)
            report.append(f"CR7机器人轨迹日志分析报告 - {file_name}")
            report.append("="*70)
            report.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            report.append("\n")
            
            # 文件基本信息
            report.append("【文件信息】")
            report.append("-"*70)
            report.append(f"文件名称: {file_name}")
            report.append(f"轨迹点数: {data['total_points']}")
            report.append(f"轨迹总时间: {data['total_time']:.3f} 秒")
            report.append(f"关节数量: {len(data['joint_names'])}")
            report.append(f"关节名称: {', '.join(data['joint_names'])}")
            report.append("\n")
            
            # 时间分析
            times = [point['time'] for point in data['trajectory_points']]
            if len(times) > 1:
                time_diffs = np.diff(times)
                report.append("【时间分析】")
                report.append("-"*70)
                report.append(f"平均时间步长: {time_diffs.mean():.6f} 秒")
                report.append(f"最大时间步长: {time_diffs.max():.6f} 秒")
                report.append(f"最小时间步长: {time_diffs.min():.6f} 秒")
                report.append("\n")
            
            # 关节数据分析
            report.append("【关节数据分析】")
            report.append("-"*70)
            for i, joint_name in enumerate(data['joint_names']):
                positions = []
                velocities = []
                accelerations = []
                
                for point in data['trajectory_points']:
                    if i < len(point['positions']):
                        positions.append(point['positions'][i])
                    if i < len(point['velocities']):
                        velocities.append(point['velocities'][i])
                    if i < len(point['accelerations']):
                        accelerations.append(point['accelerations'][i])
                
                report.append(f"\n{joint_name}:")
                report.append("-"*40)
                if positions:
                    report.append(f"位置范围: [{min(positions):.4f}, {max(positions):.4f}] rad")
                if velocities:
                    report.append(f"速度范围: [{min(velocities):.4f}, {max(velocities):.4f}] rad/s")
                if accelerations:
                    report.append(f"加速度范围: [{min(accelerations):.4f}, {max(accelerations):.4f}] rad/s²")
            
            # 保存单个文件的报告
            report_text = '\n'.join(report)
            report_file = os.path.join(output_dir, f"{os.path.splitext(file_name)[0]}_report.txt")
            
            with open(report_file, 'w', encoding='utf-8') as f:
                f.write(report_text)
            
            print(f"✓ 报告已保存: {report_file}")
        
        # 生成总体汇总报告
        self.generate_summary_report(output_dir)
    
    def generate_summary_report(self, output_dir):
        """
        生成总体汇总报告
        
        Args:
            output_dir: 输出文件夹路径
        """
        report = []
        report.append("="*70)
        report.append("CR7机器人轨迹日志分析汇总报告")
        report.append("="*70)
        report.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append("\n")
        
        # 总体统计
        report.append("【总体统计】")
        report.append("-"*70)
        report.append(f"分析文件数量: {len(self.parsed_data)}")
        
        total_points = sum(data['total_points'] for data in self.parsed_data.values())
        report.append(f"总轨迹点数: {total_points}")
        
        if self.parsed_data:
            avg_time = np.mean([data['total_time'] for data in self.parsed_data.values()])
            report.append(f"平均轨迹时间: {avg_time:.3f} 秒")
            
            # 统计文件信息
            report.append("\n【文件列表】")
            report.append("-"*70)
            for file_name, data in self.parsed_data.items():
                report.append(f"{file_name}: {data['total_points']} 点, {data['total_time']:.3f} 秒")
        
        # 保存汇总报告
        report_text = '\n'.join(report)
        summary_file = os.path.join(output_dir, "summary_report.txt")
        
        with open(summary_file, 'w', encoding='utf-8') as f:
            f.write(report_text)
        
        print(f"\n✓ 汇总报告已保存: {summary_file}")
        print("\n" + report_text)


def main():
    """主函数"""
    if len(sys.argv) > 1:
        data_dir = sys.argv[1]
    else:
        data_dir = "."
    
    print("\n" + "="*70)
    print("CR7机器人轨迹日志分析工具")
    print("="*70)
    
    # 创建分析器
    analyzer = LogDataAnalyzer(data_dir)
    
    # 查找日志文件
    analyzer.find_log_files()
    
    # 解析所有文件
    analyzer.parse_all_files()
    
    # 分析数据正确性
    analyzer.analyze_data_correctness()
    
    # 创建输出文件夹
    output_dir = "analysis_output"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"\n✓ 创建输出文件夹: {output_dir}")
    
    # 生成图表
    print("\n生成分析图表...")
    analyzer.plot_trajectory_analysis(output_dir)
    
    # 生成报告
    analyzer.generate_analysis_report(output_dir)
    
    print("\n" + "="*70)
    print("分析完成！")
    print("生成的文件:")
    print(f"  ✓ 输出文件夹: {output_dir}")
    print("  ✓ 每个轨迹文件的分析报告 (*_report.txt)")
    print("  ✓ 每个轨迹文件的分析图表 (*_analysis.png)")
    print("  ✓ 汇总报告: summary_report.txt")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
