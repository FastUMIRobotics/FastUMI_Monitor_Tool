#!/usr/bin/env bash

# 简要说明：
# - 运行本脚本后，输入编号（支持单个、逗号列表、范围），将在新终端窗口中执行对应命令。
# - 你可以在新开的终端中按 Ctrl-C 停止，或直接关闭窗口。
# - 脚本会自动尝试加载常见的 ROS 环境（如 /opt/ros/noetic、/opt/ros/melodic、~/catkin_ws）。

set -euo pipefail

# 主题前缀（根据你的实际设备 ID）
SERIAL_NUMBER=$xv_serial_number
TOPIC_PREFIX="/xv_sdk/${SERIAL_NUMBER}"
# RVIZ_PATH="/home/onestar/catkin_ws/src/xv_sdk/rviz/"
CURRENT_PATH=`pwd`
RVIZ_PATH=$CURRENT_PATH"/rviz/generated/${SERIAL_NUMBER}/"

# 构造需要执行的命令映射
declare -A CMD_MAP
CMD_MAP[1]="rostopic hz ${TOPIC_PREFIX}/slam/pose"
CMD_MAP[2]="rostopic echo ${TOPIC_PREFIX}/slam/pose"
CMD_MAP[3]="rostopic hz ${TOPIC_PREFIX}/slam/visual_pose"
CMD_MAP[4]="rostopic echo ${TOPIC_PREFIX}/slam/visual_pose"
CMD_MAP[5]="rostopic hz ${TOPIC_PREFIX}/color_camera/image_color"
CMD_MAP[6]="rostopic hz ${TOPIC_PREFIX}/fisheye_cameras/left/camera_info"
CMD_MAP[7]="rostopic hz ${TOPIC_PREFIX}/fisheye_cameras/left2/camera_info"
CMD_MAP[8]="rostopic hz ${TOPIC_PREFIX}/fisheye_cameras/right/camera_info"
CMD_MAP[9]="rostopic hz ${TOPIC_PREFIX}/fisheye_cameras/right2/camera_info"
CMD_MAP[10]="rostopic hz ${TOPIC_PREFIX}/tof_camera/image"
CMD_MAP[11]="rviz -d ${RVIZ_PATH}/four_fisheyes.rviz"
CMD_MAP[12]="rviz -d ${RVIZ_PATH}/fisheye_left.rviz"
CMD_MAP[13]="rviz -d ${RVIZ_PATH}/fisheye_left2.rviz "
CMD_MAP[14]="rviz -d ${RVIZ_PATH}/fisheye_right.rviz"
CMD_MAP[15]="rviz -d ${RVIZ_PATH}/fisheye_right2.rviz"
CMD_MAP[16]="rviz -d ${RVIZ_PATH}/rgbd_camera.rviz"
CMD_MAP[17]="rviz -d ${RVIZ_PATH}/rgb_camera.rviz"
CMD_MAP[18]="rviz -d ${RVIZ_PATH}/tof.rviz"
CMD_MAP[19]="rviz -d ${RVIZ_PATH}/slam_pose_markers.rviz"
CMD_MAP[20]="rviz -d ${RVIZ_PATH}/general.rviz"
CMD_MAP[21]="rosservice call ${TOPIC_PREFIX}/clamp/stop;rosservice call ${TOPIC_PREFIX}/clamp/start;rostopic echo ${TOPIC_PREFIX}/clamp/Data"
# 终端自动探测顺序
TERM_CANDIDATES=(
  "gnome-terminal"
  "konsole"
  "xfce4-terminal"
  "tilix"
  "xterm"
)

detect_terminal() {
  for t in "${TERM_CANDIDATES[@]}"; do
    if command -v "$t" >/dev/null 2>&1; then
      echo "$t"
      return 0
    fi
  done
  return 1
}

# 生成通用命令：加载 ROS 环境 + 目标命令
build_shell_command() {
  local user_cmd="$1"
  # 按需加载常见 ROS 环境
  local src_cmd=
""
  src_cmd+="if [ -f /opt/ros/noetic/setup.bash ]; then source /opt/ros/noetic/setup.bash; fi;"
  src_cmd+="if [ -f /opt/ros/melodic/setup.bash ]; then source /opt/ros/melodic/setup.bash; fi;"
  src_cmd+="if [ -f \"$HOME/catkin_ws/devel/setup.bash\" ]; then source \"$HOME/catkin_ws/devel/setup.bash\"; fi;"
  src_cmd+="if [ -f \"$HOME/ros_ws/devel/setup.bash\" ]; then source \"$HOME/ros_ws/devel/setup.bash\"; fi;"

  # 使用登录 shell 以便环境变量完整生效；末尾 exec bash 便于命令退出后仍停留
  echo "${src_cmd} ${user_cmd}; exec bash"
}

open_in_terminal() {
  local title="$1"
  local user_cmd="$2"
  local term_bin
  term_bin=$(detect_terminal) || {
    echo "未找到可用的终端程序（尝试：${TERM_CANDIDATES[*]}）。请安装其中之一。" >&2
    exit 1
  }

  local shell_cmd
  shell_cmd=$(build_shell_command "$user_cmd")

  case "$term_bin" in
    gnome-terminal)
      nohup gnome-terminal --title="$title" -- bash -lc "$shell_cmd" >/dev/null 2>&1 &
      ;;
    konsole)
      # -p tabtitle 设置标签标题；部分发行版需用 --hold 保持
      nohup konsole -p tabtitle="$title" -e bash -lc "$shell_cmd" >/dev/null 2>&1 &
      ;;
    xfce4-terminal)
      nohup xfce4-terminal --title="$title" --hold -x bash -lc "$shell_cmd" >/dev/null 2>&1 &
      ;;
    tilix)
      nohup tilix -t "$title" -e bash -lc "$shell_cmd" >/dev/null 2>&1 &
      ;;
    xterm)
      nohup xterm -T "$title" -e bash -lc "$shell_cmd" >/dev/null 2>&1 &
      ;;
    *)
      echo "未知终端：$term_bin" >&2
      exit 1
      ;;
  esac
}

## 已移除 image_view/rqt_image_view 的窗口尺寸与摆放辅助函数

parse_input_to_ids() {
  # 将原始输入（支持：单个数字、逗号列表、范围 1-4、以及混合）解析为去重后的编号数组
  local raw="$1"
  SELECTED_IDS=()
  declare -A seen

  local parts=()
  IFS=',' read -ra parts <<< "$raw"

  for part in "${parts[@]}"; do
    # 去除空白
    part="${part//[[:space:]]/}"
    [[ -z "$part" ]] && continue

    if [[ "$part" =~ ^[0-9]+-[0-9]+$ ]]; then
      local a b
      IFS='-' read -r a b <<< "$part"
      # 允许反序范围，如 4-1
      if (( a > b )); then
        local tmp=$a; a=$b; b=$tmp
      fi
      for ((i=a; i<=b; i++)); do
        if [[ -n "${CMD_MAP[$i]+x}" && -z "${seen[$i]+x}" ]]; then
          SELECTED_IDS+=("$i")
          seen[$i]=1
        fi
      done
    elif [[ "$part" =~ ^[0-9]+$ ]]; then
      local i=$part
      if [[ -n "${CMD_MAP[$i]+x}" && -z "${seen[$i]+x}" ]]; then
        SELECTED_IDS+=("$i")
        seen[$i]=1
      fi
    else
      echo "跳过无效输入片段：$part" >&2
    fi
  done
}

print_menu() {
  cat <<EOF
====== 选择要运行的命令（支持单个编号、逗号列表、范围）======
 1) IMU 频率   hz   slam/pose
 2) IMU数据读取   echo slam/pose
 3) SLAM 频率   hz   slam/visual_pose
 4) SLAM数据读取   echo slam/visual_pose
 5) RGB相机频率   hz   color_camera/image_color
 6) 左前鱼眼相机频率  hz   fisheye_cameras/left/camera_info
 7) 左上鱼眼相机频率  hz   fisheye_cameras/left2/camera_info
 8) 右前鱼眼相机频率  hz   fisheye_cameras/right/camera_info
 9) 右上鱼眼相机频率  hz   fisheye_cameras/right2/camera_info
10) TOF相机频率   hz   tof_camera/image
11) RViz: 四鱼眼视图           four_fisheyes.rviz
12) RViz: 左前鱼眼              fisheye_left.rviz
13) RViz: 左上鱼眼              fisheye_left2.rviz
14) RViz: 右前鱼眼              fisheye_right.rviz
15) RViz: 右上鱼眼              fisheye_right2.rviz
16) RViz: RGBD 相机             rgbd_camera.rviz
17) RViz: RGB 相机              rgb_camera.rviz
18) RViz: TOF                   tof.rviz
19) RViz: SLAM Pose标记           slam_pose_markers.rviz
20) RViz: 整体可视化                  general.rviz
21) CLAMP 数据读取   echo clamp/Data	
  0) 退出脚本2
==============================================================
提示：新开的终端窗口中按 Ctrl-C 可停止，或直接关闭窗口。
EOF
}

main() {
  while true; do
    print_menu
    read -rp "请输入编号(0-21)，可用如 1-4 或 1,2,3: " user_input
    if [[ -z "${user_input:-}" ]]; then
      echo "未输入编号，继续等待..."
      continue
    fi
    if [[ "$user_input" == "0" ]]; then
      echo "已退出脚本。"
      exit 0
    fi

    parse_input_to_ids "$user_input"
    if (( ${#SELECTED_IDS[@]} == 0 )); then
      echo "没有可执行的有效编号（范围：1-21）。"
      continue
    fi

    for num in "${SELECTED_IDS[@]}"; do
      local cmd_title="[#${num}] ${CMD_MAP[$num]}"
      open_in_terminal "$cmd_title" "${CMD_MAP[$num]}"
      echo "已在新终端窗口启动：${CMD_MAP[$num]}"

      # RViz 模式无需图像窗口自动尺寸调整
    done
  done
}

main "$@"


