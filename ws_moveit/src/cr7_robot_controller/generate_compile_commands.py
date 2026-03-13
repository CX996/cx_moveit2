#!/usr/bin/env python3
"""
生成compile_commands.json文件的脚本
用于clang工具链的代码分析
"""

import os
import json
import subprocess
import re

# 项目根目录
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
# 构建目录
BUILD_DIR = os.path.join(PROJECT_ROOT, 'build')
# 输出文件
OUTPUT_FILE = os.path.join(PROJECT_ROOT, 'compile_commands.json')

# C++源文件扩展名
CPP_EXTENSIONS = ['.cpp', '.cc', '.cxx']
# 头文件扩展名
H_EXTENSIONS = ['.h', '.hpp', '.hxx']

def find_cpp_files(directory):
    """查找目录下所有C++源文件"""
    cpp_files = []
    for root, _, files in os.walk(directory):
        for file in files:
            if any(file.endswith(ext) for ext in CPP_EXTENSIONS):
                cpp_files.append(os.path.join(root, file))
    return cpp_files

def get_include_dirs():
    """获取包含目录"""
    include_dirs = [
        os.path.join(PROJECT_ROOT, 'include'),
        os.path.join(PROJECT_ROOT, 'src'),
    ]
    
    # 尝试通过cmake获取更多包含目录
    try:
        # 确保构建目录存在
        if not os.path.exists(BUILD_DIR):
            os.makedirs(BUILD_DIR)
            print(f"创建构建目录: {BUILD_DIR}")
        
        # 检查cmake是否可用
        try:
            cmake_version = subprocess.run(
                ['cmake', '--version'],
                capture_output=True,
                text=True
            )
            print(f"CMake版本: {cmake_version.stdout.strip()}")
        except FileNotFoundError:
            print("错误: 未找到cmake命令，请确保CMake已安装并添加到系统路径")
            return include_dirs
        
        # 运行cmake获取包含目录
        print(f"在目录 {BUILD_DIR} 中运行cmake命令")
        result = subprocess.run(
            ['cmake', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON', '..'],
            cwd=BUILD_DIR,
            capture_output=True,
            text=True
        )
        
        print(f"CMake命令返回码: {result.returncode}")
        if result.stdout:
            print(f"CMake stdout: {result.stdout[:500]}...")
        if result.stderr:
            print(f"CMake stderr: {result.stderr[:500]}...")
        
        # 检查是否生成了compile_commands.json
        cmake_compile_commands = os.path.join(BUILD_DIR, 'compile_commands.json')
        if os.path.exists(cmake_compile_commands):
            print(f"从CMake生成的compile_commands.json中读取信息")
            with open(cmake_compile_commands, 'r') as f:
                cmake_commands = json.load(f)
            
            # 提取包含目录
            for cmd in cmake_commands:
                command = cmd.get('command', '')
                for part in command.split(): 
                    if part.startswith('-I'):
                        include_dir = part[2:]
                        if include_dir not in include_dirs:
                            include_dirs.append(include_dir)
        else:
            print(f"CMake未生成compile_commands.json文件")
    except Exception as e:
        print(f"获取包含目录时出错: {e}")
        import traceback
        print(traceback.format_exc())
    
    return include_dirs

def generate_compile_commands():
    """生成compile_commands.json文件"""
    # 查找所有C++源文件
    cpp_files = find_cpp_files(PROJECT_ROOT)
    print(f"找到 {len(cpp_files)} 个C++源文件")
    
    # 获取包含目录
    include_dirs = get_include_dirs()
    print(f"找到 {len(include_dirs)} 个包含目录")
    
    # 构建编译命令模板
    compile_template = [
        'g++',
        '-std=c++17',
        '-Wall',
        '-Wextra',
        '-fPIC',
    ]
    
    # 添加包含目录
    for include_dir in include_dirs:
        compile_template.append(f'-I{include_dir}')
    
    # 添加ROS 2和MoveIt相关的包含目录
    # 这些通常由ament自动处理，但为了保险起见，我们手动添加一些常见的路径
    ros2_include_paths = [
        '/opt/ros/humble/include',
        '/opt/ros/galactic/include',
        '/opt/ros/foxy/include',
        # Windows常见的ROS 2安装路径
        'C:\\opt\\ros\\humble\\include',
        'C:\\opt\\ros\\galactic\\include',
        'C:\\opt\\ros\\foxy\\include',
        # 本地工作空间的包含路径
        os.path.join(PROJECT_ROOT, '..', '..', 'install', 'include'),
        os.path.join(PROJECT_ROOT, '..', 'install', 'include'),
    ]
    
    for path in ros2_include_paths:
        if os.path.exists(path):
            compile_template.append(f'-I{path}')
    
    # 添加MoveIt相关的额外包含目录
    moveit_include_paths = [
        # 常见的MoveIt包含路径
        '/opt/ros/humble/include/moveit_core',
        '/opt/ros/humble/include/moveit_ros_planning_interface',
        'C:\\opt\\ros\\humble\\include\\moveit_core',
        'C:\\opt\\ros\\humble\\include\\moveit_ros_planning_interface',
    ]
    
    for path in moveit_include_paths:
        if os.path.exists(path):
            compile_template.append(f'-I{path}')
    
    # 生成compile_commands.json内容
    commands = []
    for cpp_file in cpp_files:
        # 构建编译命令
        command = compile_template.copy()
        command.extend([
            '-c',
            cpp_file,
            '-o',
            os.path.join(BUILD_DIR, os.path.basename(cpp_file) + '.o')
        ])
        
        # 创建命令条目
        entry = {
            'directory': PROJECT_ROOT,
            'command': ' '.join(command),
            'file': cpp_file
        }
        commands.append(entry)
    
    # 写入文件
    with open(OUTPUT_FILE, 'w') as f:
        json.dump(commands, f, indent=2)
    
    print(f"已生成compile_commands.json文件，包含 {len(commands)} 个编译命令")
    print(f"文件路径: {OUTPUT_FILE}")

if __name__ == '__main__':
    generate_compile_commands()
