# ROS主题监控脚本使用教程

## 📋 项目简介

这是一个用于监控和可视化XV SDK设备数据的交互式脚本，主要功能包括：
- 监控各种传感器的数据频率（Hz）
- 实时查看传感器数据内容
- 启动RViz可视化界面
- 支持多终端并行运行

## 🚀 快速开始

### 环境要求
- Ubuntu 20.04
- ROS Noetic


### 安装与运行

1. **确保脚本有执行权限**
   ```bash
   chmod +x run_rostopic_menu.sh
   ```

2. **运行脚本**
   ```bash
   ./run_rostopic_menu.sh
   ```

3. **基本使用**
   - 运行后会出现菜单界面
   - 输入对应编号即可执行相应功能
   - 输入 `0` 退出脚本

## 📊 功能详解

### 传感器频率监控 (1-10)

| 编号 | 功能 | 监控内容 |
|------|------|----------|
| 1 | IMU频率监控 | `slam/pose` 主题频率 |
| 2 | IMU数据读取 | `slam/pose` 主题内容 |
| 3 | SLAM频率监控 | `slam/visual_pose` 主题频率 |
| 4 | SLAM数据读取 | `slam/visual_pose` 主题内容 |
| 5 | RGB相机频率 | `color_camera/camera_info` 频率 |
| 6 | 左前鱼眼相机频率 | `fisheye_cameras/left/camera_info` 频率 |
| 7 | 左上鱼眼相机频率 | `fisheye_cameras/left2/camera_info` 频率 |
| 8 | 右前鱼眼相机频率 | `fisheye_cameras/right/camera_info` 频率 |
| 9 | 右上鱼眼相机频率 | `fisheye_cameras/right2/camera_info` 频率 |
| 10 | TOF相机频率 | `tof_camera/camera_info` 频率 |

### RViz可视化界面 (11-20)

| 编号 | 功能 | 配置文件 |
|------|------|----------|
| 11 | 四鱼眼视图 | `four_fisheyes.rviz` |
| 12 | 左前鱼眼 | `fisheye_left.rviz` |
| 13 | 左上鱼眼 | `fisheye_left2.rviz` |
| 14 | 右前鱼眼 | `fisheye_right.rviz` |
| 15 | 右上鱼眼 | `fisheye_right2.rviz` |
| 16 | RGBD相机 | `rgbd_camera.rviz` |
| 17 | RGB相机 | `rgb_camera.rviz` |
| 18 | TOF传感器 | `tof.rviz` |
| 19 | SLAM可视化 | `slam_visualization.rviz` |
| 20 | 整体可视化 | `general.rviz` |

## 💡 高级使用技巧

### 批量执行

脚本支持多种输入格式：

```bash
# 单个编号
1

# 逗号分隔
1,3,5

# 范围（支持正序和倒序）
1-5
5-1

# 混合使用
1,3-5,10
```

### 并行监控

- 可以同时启动多个监控窗口
- 每个功能在新终端窗口中运行
- 支持同时监控多个传感器

### 环境自动配置

脚本会自动加载以下ROS环境：
- `/opt/ros/noetic/setup.bash`
- `/opt/ros/melodic/setup.bash`
- `~/catkin_ws/devel/setup.bash`
- `~/ros_ws/devel/setup.bash`

## 🔧 配置说明

### 设备配置

在脚本中修改以下配置：

```bash
# 第11行：设备ID前缀
TOPIC_PREFIX="/xv_sdk/250801DR48FP25002993"

# 第12行：RViz配置文件路径
RVIZ_PATH="/home/onestar/catkin_ws/src/xv_sdk/rviz/"
```

**修改方法：**
1. 将 `250801DR48FP25002993` 替换为您的实际设备ID
2. 确认RViz配置文件路径是否正确

### 查看实际设备ID

```bash
# 查看所有ROS主题
rostopic list

# 查找XV SDK相关主题
rostopic list | grep xv_sdk
```

## 📝 使用示例

### 示例1：监控IMU数据
```bash
# 输入：1,2
# 结果：同时监控IMU频率和查看IMU数据内容
```

### 示例2：启动多个相机视图
```bash
# 输入：11-15
# 结果：启动所有鱼眼相机的RViz视图
```

### 示例3：全面监控
```bash
# 输入：1-10
# 结果：监控所有传感器的频率
```

### 示例4：混合使用
```bash
# 输入：1,3-5,11,20
# 结果：监控IMU频率、SLAM频率、RGB相机频率，启动四鱼眼视图和整体可视化
```

## ⚠️ 注意事项

1. **设备连接**：确保XV SDK设备已正确连接
2. **ROS环境**：确保ROS环境已正确配置
3. **权限问题**：确保脚本有执行权限
4. **终端窗口**：新开的终端窗口可以通过Ctrl-C停止或直接关闭
5. **配置文件**：确保RViz配置文件存在于指定路径

## 🛠️ 故障排除

### 常见问题

#### 1. 找不到终端程序
```bash
# 安装gnome-terminal
sudo apt install gnome-terminal

# 或安装其他终端
sudo apt install konsole
sudo apt install xfce4-terminal
sudo apt install tilix
```

#### 2. ROS环境未加载
```bash
# 手动source环境
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 检查ROS环境
echo $ROS_PACKAGE_PATH
```

#### 3. 设备ID不匹配
```bash
# 检查设备连接状态
lsusb | grep -i xv

# 查看实际主题名称
rostopic list | grep xv_sdk

# 修改脚本中的TOPIC_PREFIX
```

#### 4. RViz配置文件不存在
```bash
# 检查配置文件路径
ls -la /home/onestar/catkin_ws/src/xv_sdk/rviz/

# 如果路径不存在，修改脚本中的RVIZ_PATH
```

#### 5. 权限问题
```bash
# 给脚本添加执行权限
chmod +x run_rostopic_menu.sh

# 检查文件权限
ls -la run_rostopic_menu.sh
```

## 🎯 最佳实践

1. **首次使用**：建议先运行单个功能测试
2. **性能监控**：使用频率监控功能检查数据流
3. **可视化调试**：结合RViz视图进行视觉调试
4. **批量操作**：合理使用范围输入提高效率
5. **设备调试**：先监控频率，再查看数据内容
6. **多窗口管理**：合理使用多个终端窗口进行并行监控




**提示**：新开的终端窗口中按 Ctrl-C 可停止，或直接关闭窗口。
