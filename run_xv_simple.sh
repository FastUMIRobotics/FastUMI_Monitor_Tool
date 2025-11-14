#!/usr/bin/env bash

# 简化版XV设备启动脚本
# 直接通过命令行参数指定设备序列号，无需export环境变量

set -euo pipefail

# 设备序列号（命令行参数优先，然后是环境变量，最后是默认值）
DEVICE_SERIAL="${1:-${XV_DEVICE_SERIAL:-250801DR48FP25002993}}"

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GENERATE_SCRIPT="$SCRIPT_DIR/rviz/scripts/generate_configs.sh"

# 颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_header() {
    echo -e "${BLUE}[XV SIMPLE]${NC} $1"
}

# 显示使用说明
show_usage() {
    echo ""
    print_header "使用方法"
    echo ""
    echo "1. 使用默认设备:"
    echo "   ./run_xv_simple.sh"
    echo ""
    echo "2. 指定设备序列号:"
    echo "   ./run_xv_simple.sh 250801DR48FP25002993"
    echo ""
    echo "3. 使用环境变量:"
    echo "   export XV_DEVICE_SERIAL=250801DR48FP25002993"
    echo "   ./run_xv_simple.sh"
    echo ""
    echo "当前设备: $DEVICE_SERIAL"
}

# 生成配置文件
generate_config() {
    print_info "正在为设备 $DEVICE_SERIAL 生成配置文件..."
    
    if [ -f "$GENERATE_SCRIPT" ]; then
        if "$GENERATE_SCRIPT" "$DEVICE_SERIAL" >/dev/null 2>&1; then
            print_info "✓ 配置文件生成成功"
            return 0
        else
            print_warning "⚠ 配置文件生成失败"
            return 1
        fi
    else
        print_warning "⚠ 未找到配置文件生成脚本"
        return 1
    fi
}

# 启动原始脚本
start_original_script() {
    print_info "启动原始菜单脚本..."
    
    # 临时设置环境变量
    XV_DEVICE_SERIAL="$DEVICE_SERIAL" "$SCRIPT_DIR/run_rostopic_menu.sh"
}

# 主函数
main() {
    print_header "XV设备启动器"
    print_info "当前设备: $DEVICE_SERIAL"
    
    # 生成配置文件
    generate_config
    echo ""
    
    # 启动原始脚本
    start_original_script
}

# 如果直接运行脚本
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    # 检查是否需要显示帮助
    if [[ "${1:-}" == "-h" ]] || [[ "${1:-}" == "--help" ]]; then
        show_usage
        exit 0
    fi
    
    main "$@"
fi
