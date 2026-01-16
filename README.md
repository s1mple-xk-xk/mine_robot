# 基于 ROS 2 和 Navigation 2 自动巡检机器人

## 1. 项目介绍

本项目是一个基于 ROS 2 和 Navigation 2 的自动巡检机器人系统。机器人能够按照预设的路径点进行自动巡逻，并在到达每个目标点时记录图像信息。

### 主要功能

- **自动导航巡逻**：基于 Navigation 2 框架，实现自主导航到多个预设目标点
- **语音播报**：使用 espeak-ng 进行中文语音播报，实时反馈机器人状态
- **图像记录**：在到达目标点时自动记录当前位姿和相机图像
- **位姿管理**：支持初始位姿设置和实时位姿获取

### 项目结构

```
src/
├── autopatrol_robot/          # 自动巡逻机器人主包
│   ├── autopatrol_robot/
│   │   ├── patrol_node.py     # 巡逻导航节点
│   │   └── speaker.py         # 语音播报服务节点
│   ├── config/
│   │   └── patrol_config.yaml # 巡逻配置参数
│   └── launch/
│       └── autopatrol.launch.py # 启动文件
├── autopatrol_interfaces/     # ROS 2 接口定义包
│   └── srv/
│       └── SpeachText.srv     # 语音文本服务接口
├── mine_nav2/                 # Navigation 2 配置包
│   ├── config/
│   │   └── nav2_params.yaml   # Nav2 参数配置
│   ├── map/                   # 地图文件
│   └── launch/
│       └── nav2.launch.py     # Nav2 启动文件
├── mine_aplication/           # 应用工具包
│   └── autopatrol_robot/      # 导航相关工具
└── mine_turtlebot3_gazebo/ # TurtleBot3 Gazebo 仿真
```

## 2. 使用方法

### 2.1 安装依赖

#### 系统依赖

```bash
# 安装 espeak-ng（语音合成）
sudo apt-get update
sudo apt-get install espeak-ng espeak-ng-data

# 安装 Python 依赖
pip3 install espeakng opencv-python
```

#### ROS 2 依赖

确保已安装以下 ROS 2 包：

- `rclpy` - ROS 2 Python 客户端库
- `nav2_simple_commander` - Navigation 2 简单导航接口
- `nav2_bringup` - Navigation 2 启动包
- `geometry_msgs` - 几何消息类型
- `sensor_msgs` - 传感器消息类型
- `cv_bridge` - OpenCV 与 ROS 图像转换
- `tf2_ros` - TF2 变换库
- `tf_transformations` - TF 变换工具

#### 编译项目

```bash
# 在工作空间根目录下
cd /home/xk/study/mine_slam
colcon build --symlink-install
source install/setup.bash
```

### 2.2 配置参数

编辑 `autopatrol_robot/config/patrol_config.yaml` 文件，配置巡逻参数：

```yaml
patrol_node:
  ros__parameters:
    # 是否使用仿真时间（仿真环境请设为 true）
    use_sim_time: true

    # 初始位姿 [x, y, yaw(rad)]
    initial_point: [-2.0, -0.5, 0.0]

    # 目标点列表，按 [x, y, yaw(rad)] 依次排列
    target_points: [-1.0, 0.0, 0.0, 1.0, 1.0, 1.57]

    # 图片保存路径（需要以 / 结尾）
    image_save_path: 'images/'

    # 图像话题（与仿真/相机模型一致）
    image_topic: '/camera/image_raw'
```

### 2.3 运行

#### 启动 Navigation 2

首先启动 Navigation 2 导航系统：

```bash
ros2 launch mine_nav2 nav2.launch.py
```

#### 启动自动巡逻

在另一个终端中启动自动巡逻节点：

```bash
ros2 launch autopatrol_robot autopatrol.launch.py
```

或者单独运行节点：

```bash
# 启动语音服务
ros2 run autopatrol_robot speaker

# 启动巡逻节点
ros2 run autopatrol_robot patrol_node --ros-args --params-file src/autopatrol_robot/config/patrol_config.yaml
```

### 2.4 工作流程

1. **初始化**：机器人设置初始位姿并等待 Navigation 2 激活
2. **语音提示**：播报"正在初始化位置"和"位置初始化完成"
3. **巡逻循环**：
   - 依次导航到配置的每个目标点
   - 到达目标点后播报"已到达目标点"
   - 记录当前位姿和相机图像
   - 图像文件名格式：`image_{x}_{y}.png`
4. **完成**：所有目标点访问完成后，循环继续或退出

## 3. 作者

- **维护者**：xk
- **GitHub**：[@s1mple-xk-xk](https://github.com/s1mple-xk-xk)