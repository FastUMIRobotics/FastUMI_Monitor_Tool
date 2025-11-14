#!/bin/bash

# RViz配置文件生成脚本
# 使用环境变量 XV_DEVICE_SERIAL 替换模板中的占位符

set -uo pipefail

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
TEMPLATE_DIR="$PROJECT_ROOT/rviz/templates"
# GENERATED_DIR 将在 check_environment 函数中设置

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查环境变量或命令行参数
check_environment() {
    # 支持命令行参数
    if [ -n "${1:-}" ]; then
        XV_DEVICE_SERIAL="$1"
    fi
    
    if [ -z "${XV_DEVICE_SERIAL:-}" ]; then
        print_error "设备序列号未设置"
        echo ""
        echo "使用方法："
        echo "  1. 命令行参数: $0 你的设备序列号"
        echo "  2. 环境变量: export XV_DEVICE_SERIAL=你的设备序列号"
        echo ""
        echo "示例："
        echo "  $0 250801DR48FP25002993"
        echo "  export XV_DEVICE_SERIAL=250801DR48FP25002993 && $0"
        exit 1
    fi
    
    print_info "使用设备序列号: $XV_DEVICE_SERIAL"
    
    # 设置输出目录
    GENERATED_DIR="$PROJECT_ROOT/rviz/generated/${XV_DEVICE_SERIAL}"
}

# 检查目录和文件
check_directories() {
    if [ ! -d "$TEMPLATE_DIR" ]; then
        print_error "模板目录不存在: $TEMPLATE_DIR"
        exit 1
    fi
    
    # 创建生成目录
    mkdir -p "$GENERATED_DIR"
    print_info "输出目录: $GENERATED_DIR"
}

# 生成配置文件
generate_configs() {
    local count=0
    local errors=0
    
    print_info "开始生成配置文件..."
    
    # 处理所有模板文件
    for template_file in "$TEMPLATE_DIR"/*.template; do
        if [ ! -f "$template_file" ]; then
            continue
        fi
        
        local basename_file=$(basename "$template_file" .template)
        local output_file="$GENERATED_DIR/$basename_file"
        
        print_info "处理: $basename_file"
        
        # 使用envsubst替换环境变量
        if XV_DEVICE_SERIAL="$XV_DEVICE_SERIAL" envsubst < "$template_file" > "$output_file" 2>/dev/null; then
            print_info "  ✓ 生成成功: $output_file"
            ((count++))
        else
            print_error "  ✗ 生成失败: $basename_file"
            ((errors++))
            # 继续处理下一个文件，不要退出
        fi
    done
    
    echo ""
    print_info "生成完成！"
    print_info "成功: $count 个文件"
    if [ $errors -gt 0 ]; then
        print_warning "失败: $errors 个文件"
    fi
}

# 验证生成的文件
verify_configs() {
    print_info "验证生成的配置文件..."
    
    local invalid_files=()
    
    for config_file in "$GENERATED_DIR"/*.rviz; do
        if [ ! -f "$config_file" ]; then
            continue
        fi
        
        local basename_file=$(basename "$config_file")
        
        # 检查是否还有未替换的占位符
        if grep -q '\${XV_DEVICE_SERIAL}' "$config_file"; then
            invalid_files+=("$basename_file")
        fi
        
        # 检查是否包含设备序列号
        if ! grep -q "$XV_DEVICE_SERIAL" "$config_file"; then
            print_warning "文件 $basename_file 中未找到设备序列号"
        fi
    done
    
    if [ ${#invalid_files[@]} -gt 0 ]; then
        print_error "以下文件仍包含未替换的占位符:"
        for file in "${invalid_files[@]}"; do
            echo "  - $file"
        done
        return 1
    fi
    
    print_info "所有配置文件验证通过！"
    return 0
}

# 显示使用说明
show_usage() {
    echo ""
    print_info "生成的配置文件位于: $GENERATED_DIR"
    echo ""
    echo "使用方法："
    echo "  1. 设置环境变量: export XV_DEVICE_SERIAL=你的设备序列号"
    echo "  2. 运行生成脚本: $0"
    echo "  3. 启动rviz: ./run_rostopic_menu.sh"
    echo ""
    echo "示例："
    echo "  export XV_DEVICE_SERIAL=250801DR48FP25002993"
    echo "  $0"
    echo "  ./run_rostopic_menu.sh"
}

# 主函数
main() {
    echo "=========================================="
    echo "    RViz 配置文件生成工具"
    echo "=========================================="
    
    check_environment "$@"
    check_directories
    generate_configs
    
    if verify_configs; then
        show_usage
        exit 0
    else
        exit 1
    fi
}

# 如果直接运行脚本
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
