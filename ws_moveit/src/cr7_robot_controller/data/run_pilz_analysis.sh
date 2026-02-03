#!/bin/bash
# PILZ焊接路径分析快速启动脚本

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "PILZ焊接路径分析工具"
echo "=========================================="
echo ""

# 检查Python
if ! command -v python3 &> /dev/null; then
    echo "❌ 错误: 找不到python3"
    exit 1
fi

# 检查依赖
echo "检查依赖..."
missing_packages=()

for package in pandas numpy matplotlib; do
    python3 -c "import $package" 2>/dev/null
    if [ $? -ne 0 ]; then
        missing_packages+=("$package")
    fi
done

if [ ${#missing_packages[@]} -gt 0 ]; then
    echo "⚠️  缺少依赖包: ${missing_packages[@]}"
    echo "安装命令: pip3 install ${missing_packages[@]}"
    echo ""
    read -p "是否现在安装? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        pip3 install "${missing_packages[@]}"
    else
        echo "继续运行（可能会失败）..."
    fi
fi

echo "✓ 依赖检查完成"
echo ""

# 检查数据文件
echo "检查数据文件..."
waypoint_files=$(ls pilz_welding_waypoints_*.csv 2>/dev/null | wc -l)
trajectory_files=$(ls pilz_*_trajectory_*.csv 2>/dev/null | wc -l)

echo "  焊接路径点文件: $waypoint_files 个"
echo "  PILZ轨迹文件: $trajectory_files 个"
echo ""

if [ $waypoint_files -eq 0 ] || [ $trajectory_files -eq 0 ]; then
    echo "❌ 错误: 缺少必要的数据文件"
    echo "请先运行CR7机器人控制程序并执行PILZ焊接路径测试"
    echo ""
    echo "步骤:"
    echo "  1. ros2 launch cr7_controller cr7_controller.launch.py"
    echo "  2. 选择菜单选项 9 (PILZ规划器测试)"
    echo "  3. 选择选项 6 (PILZ焊接点位路径测试)"
    exit 1
fi

echo "✓ 找到必要的数据文件"
echo ""

# 运行分析
echo "运行分析..."
python3 analyze_pilz_welding.py

echo ""
echo "=========================================="
echo "分析完成!"
echo "=========================================="
echo ""
echo "输出文件:"
echo "  📊 pilz_welding_analysis.png"
echo "  📊 pilz_linearity_check.png"
echo "  📄 pilz_welding_analysis_report.txt"
echo ""
echo "使用图片查看器打开PNG文件："
echo "  eog pilz_welding_analysis.png &"
echo "  eog pilz_linearity_check.png &"
echo ""
echo "查看文本报告："
echo "  cat pilz_welding_analysis_report.txt"
echo ""
