#!/bin/bash

# 设备管理工具
# 用于管理不同的XV设备序列号

set -euo pipefail

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
CONFIG_FILE="$PROJECT_ROOT/rviz/config/device_list.conf"
GENERATE_SCRIPT="$PROJECT_ROOT/rviz/scripts/generate_configs.sh"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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

print_header() {
    echo -e "${BLUE}[DEVICE MANAGER]${NC} $1"
}

# 初始化配置文件
init_config() {
    local config_dir="$(dirname "$CONFIG_FILE")"
    mkdir -p "$config_dir"
    
    if [ ! -f "$CONFIG_FILE" ]; then
        cat > "$CONFIG_FILE" << EOF
# XV设备配置文件
# 格式: DEVICE_ID=SERIAL_NUMBER:DEVICE_NAME:DESCRIPTION

# 默认设备
DEFAULT=250801DR48FP25002993:XV-Default:默认设备

# 示例设备（请根据实际情况修改）
# DEVICE_001=250801DR48FP25002993:XV-001:生产设备
# DEVICE_002=250801DR48FP25003001:XV-002:测试设备
# DEVICE_003=250801DR48FP25003015:XV-003:开发设备
EOF
        print_info "已创建默认配置文件: $CONFIG_FILE"
    fi
}

# 解析设备配置
parse_device_config() {
    local device_id="$1"
    local line
    line=$(grep "^${device_id}=" "$CONFIG_FILE" 2>/dev/null || echo "")
    
    if [ -z "$line" ]; then
        return 1
    fi
    
    # 解析格式: DEVICE_ID=SERIAL:NAME:DESCRIPTION
    IFS='=' read -r _ device_info <<< "$line"
    IFS=':' read -r serial name description <<< "$device_info"
    
    echo "$serial|$name|$description"
}

# 列出所有设备
list_devices() {
    print_header "可用设备列表"
    echo ""
    
    if [ ! -f "$CONFIG_FILE" ]; then
        print_error "配置文件不存在: $CONFIG_FILE"
        return 1
    fi
    
    local current_device="${XV_DEVICE_SERIAL:-未设置}"
    
    while IFS='=' read -r device_id device_info; do
        # 跳过注释和空行
        if [[ "$device_id" =~ ^#.*$ ]] || [ -z "$device_id" ]; then
            continue
        fi
        
        IFS=':' read -r serial name description <<< "$device_info"
        
        # 标记当前设备
        local marker=""
        if [ "$serial" = "$current_device" ]; then
            marker=" (当前)"
        fi
        
        printf "%-12s %-25s %-15s %s%s\n" \
            "$device_id" "$serial" "$name" "$description" "$marker"
    done < "$CONFIG_FILE"
    
    echo ""
    print_info "当前设备: $current_device"
}

# 切换设备
switch_device() {
    local device_id="$1"
    
    if [ -z "$device_id" ]; then
        print_error "请指定设备ID"
        return 1
    fi
    
    local device_info
    if ! device_info=$(parse_device_config "$device_id"); then
        print_error "设备ID不存在: $device_id"
        return 1
    fi
    
    IFS='|' read -r serial name description <<< "$device_info"
    
    print_info "切换到设备: $name ($serial)"
    
    # 设置环境变量
    export XV_DEVICE_SERIAL="$serial"
    
    # 生成配置文件
    if [ -f "$GENERATE_SCRIPT" ]; then
        print_info "正在生成配置文件..."
        if "$GENERATE_SCRIPT" >/dev/null 2>&1; then
            print_info "✓ 配置文件生成成功"
        else
            print_error "✗ 配置文件生成失败"
            return 1
        fi
    else
        print_warning "未找到配置文件生成脚本"
    fi
    
    print_info "设备切换完成！"
    print_info "当前设备: $name ($serial)"
    echo ""
    print_info "现在可以运行: ./run_rostopic_menu.sh"
}

# 添加新设备
add_device() {
    local device_id="$1"
    local serial="$2"
    local name="${3:-$device_id}"
    local description="${4:-新设备}"
    
    if [ -z "$device_id" ] || [ -z "$serial" ]; then
        print_error "请提供设备ID和序列号"
        return 1
    fi
    
    # 检查设备ID是否已存在
    if grep -q "^${device_id}=" "$CONFIG_FILE"; then
        print_error "设备ID已存在: $device_id"
        return 1
    fi
    
    # 检查序列号是否已存在
    if grep -q ":$serial:" "$CONFIG_FILE"; then
        print_error "设备序列号已存在: $serial"
        return 1
    fi
    
    # 添加设备配置
    echo "${device_id}=${serial}:${name}:${description}" >> "$CONFIG_FILE"
    print_info "已添加设备: $device_id ($name)"
}

# 删除设备
remove_device() {
    local device_id="$1"
    
    if [ -z "$device_id" ]; then
        print_error "请指定设备ID"
        return 1
    fi
    
    if ! grep -q "^${device_id}=" "$CONFIG_FILE"; then
        print_error "设备ID不存在: $device_id"
        return 1
    fi
    
    # 备份原文件
    cp "$CONFIG_FILE" "${CONFIG_FILE}.bak"
    
    # 删除设备配置
    grep -v "^${device_id}=" "$CONFIG_FILE" > "${CONFIG_FILE}.tmp"
    mv "${CONFIG_FILE}.tmp" "$CONFIG_FILE"
    
    print_info "已删除设备: $device_id"
}

# 交互式设备选择
interactive_switch() {
    print_header "交互式设备选择"
    echo ""
    
    list_devices
    echo ""
    
    read -rp "请输入要切换的设备ID: " device_id
    
    if [ -z "$device_id" ]; then
        print_warning "未选择设备"
        return 1
    fi
    
    switch_device "$device_id"
}

# 显示帮助信息
show_help() {
    cat << EOF
XV设备管理工具

用法: $0 [命令] [参数]

命令:
  list                   列出所有可用设备
  switch <device_id>     切换到指定设备
  add <id> <serial> [name] [desc]  添加新设备
  remove <device_id>     删除设备
  interactive            交互式设备选择
  help                   显示此帮助信息

示例:
  $0 list
  $0 switch DEFAULT
  $0 switch DEVICE_001
  $0 add DEVICE_004 250801DR48FP25003020 XV-004 新设备
  $0 remove DEVICE_003
  $0 interactive

配置文件: $CONFIG_FILE
EOF
}

# 主函数
main() {
    # 初始化配置
    init_config
    
    local command="${1:-help}"
    
    case "$command" in
        "list")
            list_devices
            ;;
        "switch")
            if [ -z "${2:-}" ]; then
                print_error "请指定设备ID"
                show_help
                exit 1
            fi
            switch_device "$2"
            ;;
        "add")
            if [ -z "${2:-}" ] || [ -z "${3:-}" ]; then
                print_error "请提供设备ID和序列号"
                show_help
                exit 1
            fi
            add_device "$2" "$3" "${4:-}" "${5:-}"
            ;;
        "remove")
            if [ -z "${2:-}" ]; then
                print_error "请指定设备ID"
                show_help
                exit 1
            fi
            remove_device "$2"
            ;;
        "interactive")
            interactive_switch
            ;;
        "help"|"-h"|"--help")
            show_help
            ;;
        *)
            print_error "未知命令: $command"
            show_help
            exit 1
            ;;
    esac
}

# 如果直接运行脚本
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
