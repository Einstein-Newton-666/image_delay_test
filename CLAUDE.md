# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

ROS 2 工作空间，用于对比测试多种图像传输方式的延迟和 CPU 占用率：
- **ros2 image_transport**：标准 ROS 2 图像传输（延迟 2-11ms，CPU 50%）
- **ros2_shm_msgs**：基于 ROS 2 loaned API 的零拷贝共享内存消息（延迟 2-5ms，CPU 20%）
- **shm_video_transmission**：基于 Boost.Interprocess 的共享内存视频传输（延迟 0.4-1.8ms，CPU 20%）
- **UltraMultiThread (UMT)**：基于 pybind11 的进程内命名对象通信（延迟 0.04-0.09ms，CPU 15%）
- **iceoryx 直接通信**：基于 iceoryx 原生 API 的共享内存通信（延迟 0.7-1.0ms）
- **autoaim_shm_image_transport**：独立头文件包，移植自 auto_aim 的 `shm_open` + `mmap` + futex 单槽共享内存取图；mode=7 支持直接写 shm slot（p50 约 0.045ms）和源 Mat 拷贝到 shm（p50 约 1.389ms）

测试平台：AMD Ryzen 7 5800H，以 200Hz 发布 1920x1024 图像。

## 构建命令

```bash
# 构建整个工作空间
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --parallel-workers 2

# 构建单个包（如 image_test）
colcon build --symlink-install --packages-select image_test --parallel-workers 2

# 加载工作空间环境
source install/setup.bash
```

## 运行测试

```bash
# 启动图像传输延迟测试
ros2 launch image_test image_test.launch.py

# 启动参数说明：
#   mode: 1=ros2, 2=shm_video_transmission, 3=UltraMultiThread, 4=loaned_msg+shm_msg, 5=iceoryx直接通信, 6=raw unique_ptr, 7=auto_aim POSIX shm ring（默认4）
#   image_pub_frequency: 发布频率 Hz（默认 200）
#   queue_size: 发布队列大小（默认 1）
#   copy_image: 接收时是否拷贝图像（默认 false）
#   generate_in_transport_buffer: 是否直接在目标传输 buffer 生成图像（默认 true；mode=7 为直接写 shm slot）
#   move_image: 发布时是否移动图像（默认 true）

# mode=5 需要先启动 RouDi（使用自定义配置支持大图像）
# 注意：不要设置 RMW_IMPLEMENTATION=rmw_iceoryx_cpp，否则延迟会升高到 20ms
unset RMW_IMPLEMENTATION
/opt/ros/humble/bin/iox-roudi -c roudi_config.toml &

# 运行 shm_video_transmission 独立 demo
cd install/shm_video_transmission/lib/shm_video_transmission
./sender <通道名> <宽> <高> <视频文件路径> <帧率>
./receiver <通道名>
```

## 架构说明

### 包结构

- **image_test**：主测试包。包含 `image_pub` 和 `image_sub` 两个 ROS 2 Component，通过 `mode` 参数切换多种传输方式。

- **UltraMultiThread (UMT)**：头文件库（`umt/ObjManager.hpp`，`umt/Message.hpp`），通过全局 `std::unordered_map` 存储命名共享对象实现进程内发布/订阅。使用 pybind11 支持 Python 互操作。**关键约束**：UMT 仅在 ROS 2 Component 中可用，普通独立节点无法找到发布者/订阅者。

- **shm_video_transmission**：基于 Boost.Interprocess 的共享内存视频传输。`VideoSender` 创建共享内存，`VideoReceiver` 映射共享内存。支持零拷贝（`toCvShare()`）和拷贝（`toCvCopy()`）两种接收模式。仅支持 CV_8UC3 格式。

- **ros2_shm_msgs**：提供多种大小（8k/512k/1m/2m/4m/8m）的零拷贝图像和点云消息定义，包含 OpenCV、PCL、Open3D 的转换工具。需要 DDS 配置共享内存支持（CycloneDDS+iceoryx 或 FastDDS）。

- **autoaim_shm_image_transport**：独立 header-only 包，位于 `src/autoaim_shm_image_transport/`。移植自 `/home/gaoyuan/auto_aim/src/rm_utils/include/rm_utils/` 的 `shared_memory_ring.hpp`、`shm_image_publisher.hpp`、`shm_image_subscriber.hpp`，容量改为 1920x1024x3，命名空间为 `autoaim_shm_image_transport`。`image_test` 的 mode=7 依赖该包，并用两个 component container 进程验证跨进程取图；`generate_in_transport_buffer=true` 时发布端通过 `borrow_frame()` 直接写 shm slot，`false` 时保留源 Mat memcpy 到 shm 的旧口径；`copy_image=false` 为订阅端零拷贝包装，`copy_image=true` 为订阅端 clone。

### 关键约束

- UMT 必须使用 `rclcpp_components` 的 Component 容器运行，封装成普通节点无法工作
- ROS 2 Component 默认单线程运行，每个发布者/订阅者需要手动开独立线程
- shm_video_transmission 的 `send()` 会将输入图像 resize 为构造时指定的尺寸
- mode=7 是单槽共享内存 ring；发布端直接写 shm slot 时延迟最低，源 Mat copy 路径用于和旧结果对比。订阅端若长期持有零拷贝 Mat，会阻塞发布端覆盖该槽，业务处理较慢时应使用 `copy_image:=true`
- ros2_shm_msgs 零拷贝需要配置 DDS 共享内存（配置文件位于 `src/ros2_shm_msgs/config/`）
- mode=5 (iceoryx 直接通信) 需要 RouDi 守护进程，且必须使用 ROS 2 自带的 RouDi（`/opt/ros/humble/bin/iox-roudi`），系统自带的 iceoryx 1.x 版本不兼容
- mode=5 需要自定义 RouDi 配置（`roudi_config.toml`）将最大内存块设为 12MB 以容纳 1920x1024 图像
- `rmw_iceoryx_cpp` 在高频下（100Hz+）对大图像消息有约 20ms 固有开销，不推荐用于大图像高频传输

### 消息类型

- `ImagePack`：自定义结构体，封装 `cv::Mat` + `builtin_interfaces::msg::Time`，用于 UMT 传输
- `shm_msgs::msg::Image8m`：共享内存图像消息（8MB 容量），配合 loaned API 使用
- `sensor_msgs::msg::Image`：标准 ROS 2 图像消息，用于基准对比
- `IceoryxImageHeader`：iceoryx 直接通信使用的图像头结构体（`image_pub.hpp`），包含时间戳、宽高、步长、编码信息，后接原始图像数据
