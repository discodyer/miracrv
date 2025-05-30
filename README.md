# MiracRV

琪迹小车，基于RISC-V开发板的ROS小车

组装和说明文档请查看：[MiracRV_docs](https://github.com/discodyer/miracrv_docs) [Gitlab](https://isrc.iscas.ac.cn/gitlab/ros2-rv/miracrv_docs)

## 仓库包含的软件包

- `miracrv_msgs` 提供所有消息格式

- `miracrv_driver` 提供小车底盘控制接口

- `miracrv_bringup` 提供launch脚本，一键启动小车节点

- `miracrv_description` 提供小车模型等

## ArduRover参数设置

我们使用ArduPilot的EKF3进行位姿估计，EKF3依赖外部传感器，比如GPS、光流、车轮编码器、视觉里程计、激光雷达等的数据，否则无法在Guided模式下解锁。

下面的配置是在室内情况下，无GPS，使用激光雷达进行位姿估计。如果是室外使用，可以加装GPS模块并按照ArduPilot相关文档配置。

- `FRAME_TYPE` = 2 设置机架类型为 OmniX
- `SERVO1_FUNCTION` = 33 设置为 Motor1
- `SERVO2_FUNCTION` = 34 设置为 Motor2
- `SERVO3_FUNCTION` = 35 设置为 Motor3
- `SERVO4_FUNCTION` = 36 设置为 Motor4

- `GPS_TYPE` = 0 禁用 GPS
- `ARMING_CHECK` = 388598 禁用 GPS 相关解锁检查
- `AHRS_GPS_USE` = 0 禁用 GPS
- `EK3_GPS_CHECK` = 0 禁用 GPS 检查
- `EK3_GSF_RUN_MASK` = 0 禁用 EKF-GSF
- `EK3_GSF_USE_MASK` = 0 禁用 EKF-GSF
- `AHRS_OPTIONS` = 3 禁止 EKF3 自动回退到 DCM

- `EK3_SRC_OPTIONS` = 0 速度融合选项，因为没有其他速度源所以为 0
- `AHRS_EKF_TYPE` = 3 启用 EKF3
- `EK2_ENABLE` = 0 禁用 EKF2
- `EK3_ENABLE` = 1 启用 EKF3
- `EK3_SRC1_POSXY` = 6 将位置水平源设置为 ExternalNAV
- `EK3_SRC1_POSZ` = 1 将位置垂直源设置为气压计
- `EK3_SRC1_VELXY` = 6 将速度水平源设置为 ExternalNAV
- `EK3_SRC1_VELZ` = 6 将垂直速度源设置为 ExternalNAV
- `EK3_SRC1_YAW` = 6 将偏航源设置为 ExternalNAV
- `VISO_TYPE` = 1 启用视觉里程计类型为 MavLink

其他参数如遥控器接收机，电调类型等需要根据购买的型号不同来修改相关参数，并且在修改完上述参数后还需要常规的进行校准加速度计、校准水平、校准遥控器、校准磁罗盘等操作，可以使用MissionPlanner地面站完成。

## 安装和构建软件包

首先，创建一个工作区，比如

```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
```

随后克隆本仓库和镭神雷达仓库

```bash
git clone -b M10P/N10P https://github.com/Lslidar/Lslidar_ROS2_driver.git
git clone https://github.com/discodyer/miracrv.git
```

构建镭神雷达的时候可能需要额外安装这个包

```bash
sudo apt-get install libpcap0.8-dev
```

然后构建工作区

```bash
cd ..
colcon build
source ./install/setup.bash
```

运行整个代码只需要执行两个launch文件

```bash
# 启动 mavros、lslidar、miracrv_driver、tf_to_odom、cartographer
ros2 launch miracrv_bringup miracrv.launch.py
# 启动 navigation2
ros2 launch miracrv_bringup navigation.launch.py
```

![alt text](image.png)

## 软件功能介绍

- `mavros` 这个节点的作用是和小车通信，这个节点提供了若干服务和话题和小车通信，首先需要往外部位姿估计接口传入里程计信息（/mavros/vision_pose/pose geometry_msgs:PoseStamped）然后小车内的EKF3就会在话题(/mavros/local_position/odom)里发布融合后的里程计信息，随后通过一系列解锁操作，就可以通过速度控制接口控制小车（/mavros/setpoint_raw/local mavros_msgs:PositionTarget）

- `tf_to_odom.py` 这个节点的作用是把cartographer发布的tf坐标变换转换成发布给小车的外部里程计信息（/mavros/vision_pose/pose geometry_msgs:PoseStamped）其中需要进行一些坐标变换，转换成NED坐标系传给小车。需要注意的是这个话题(/mavros/local_position/odom)发布的是EKF3融合后的里程计信息，如果直接再喂给cartographer的话，由于存在延迟，会导致cartographer中的定位出现像游戏卡顿一样的回退。

- `Lslidar_driver` 激光雷达的驱动包，负责提供/scan话题

- `Navigation2` 提供导航功能，但是由于未知原因，只能单独启动，不能和其他节点合并到一个launch文件内启动

- `MiracRV_Driver` 提供和小车交互的一系列操作，自动切换模式和解锁等，并且监听话题`/miracrv/cmd_vel`里的`TwistStamped`消息，然后转换成（/mavros/setpoint_raw/local mavros_msgs:PositionTarget）控制小车移动。

## 后续规划

- 由于缺少里程计，后面计划重写一个小车底层驱动板，能输出IMU和编码器的里程计信息，并且可以分别控制四个电机

- navgation2的配置有一些问题，侧向移动和后退都有问题

- 添加小车模型和仿真环境

- 移植到openEuler上运行

- 移植到RISC-V开发板上运行
