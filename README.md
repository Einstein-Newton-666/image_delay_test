# ros2图像传输延迟测试

测试对象：ros2 image_transport，ros2 borrow_msg + shm_msg，shm_video_transmission，UltraMultiThread，iceoryx直接通信

测试平台：AMD Ryzen 7 5800H

测试方式：在ros2 component 中以200hz发布1920*1024大小的图像，查看图像传输延迟和cpu占用率 （未关闭ros info）

## 测试结果（优化后，200Hz 长时间测试）

| 图像传输方式                         | mode | 传输延迟      | cpu占用率 |
| :----------------------------------- | ---- | ------------- | --------- |
| ros2 image_transport(queue_size=1)   | 1    | 3.0~3.9ms     | 16%       |
| ros2 borrow_msg + shm_msg            | 4    | 2.3~3.3ms     | 8%        |
| shm_video_transmission               | 2    | 1.3~1.9ms     | 9%        |
| UltraMultiThread                     | 3    | 0.04~0.07ms   | 0.3%      |
| iceoryx                              | 5    | 0.24~0.52ms   | 8%        |

## 优化措施

### 跨模式通用优化

1. **预分配 cv::Mat**（mode 1/2/3）：首次分配后复用，避免每帧 5.9MB 的 malloc+memset。实际相机场景中，图像来自相机回调，无需此优化。
2. **修复编码不匹配**（mode 1 订阅端）：`toCvShare(msg, "rgb8")` 因编码不同退化为 `toCvCopy` + 颜色转换，改为 `toCvShare(msg)` 真正零拷贝。
3. **编译优化**：`-O3 -march=native` 启用 AVX2 指令集加速 memcpy。
4. **直接写入 loaned 消息**（mode 4）：跳过 CvImage 中间层，用 `cv::Mat wrapper` 直接引用 loaned 消息的共享内存 buffer。

### 各模式优化详情

| 模式 | 优化前 | 优化后 | 主要优化 |
|------|--------|--------|----------|
| mode=1 | 2~11ms | 3.0~3.9ms | 修复编码不匹配，消除订阅端颜色转换拷贝 |
| mode=2 | 0.4~1.8ms | 1.3~1.9ms | 预分配 Mat |
| mode=3 | 0.04~0.09ms | 0.04~0.07ms | move 语义 + 预分配 Mat |
| mode=4 | 2~5ms | 2.3~3.3ms | 直接写入 loaned 消息，消除中间 buffer |
| mode=5 | 0.7~1.0ms | 0.24~0.52ms | -O3 编译优化 + 预分配 Mat |

## iceoryx 直接通信 (mode=5)

使用 iceoryx 原生 C++ API（`iox::popo::UntypedPublisher`/`UntypedSubscriber`）直接进行共享内存通信，绕过 ROS 2 DDS 中间件和 rmw 层。

### 前置条件

需要启动 RouDi 守护进程，并使用自定义配置以支持大尺寸图像消息（默认内存池最大 4MB，不足以容纳 1920x1024 图像）：

```bash
# 使用自定义配置启动 RouDi（最大 chunk 12MB）
/opt/ros/humble/bin/iox-roudi -c /home/gaoyuan/image_delay_test/roudi_config.toml
```

> 注意：必须使用 ROS 2 自带的 RouDi（`/opt/ros/humble/bin/iox-roudi`），系统自带的 iceoryx 1.x 版本与 ROS 2 Humble 的 iceoryx 2.0.5 不兼容。

### 运行

```bash
# 确保没有设置 RMW_IMPLEMENTATION（设了会导致延迟升高到 20ms）
unset RMW_IMPLEMENTATION

# 启动 RouDi
/opt/ros/humble/bin/iox-roudi -c /home/gaoyuan/image_delay_test/roudi_config.toml &

# 运行测试
ros2 launch image_test image_test.launch.py mode:=5 image_pub_frequency:=200
```

> 注意：不要设置 `RMW_IMPLEMENTATION=rmw_iceoryx_cpp`。mode 5 直接调用 iceoryx 原生 API，不经过任何 RMW 层。设置了该变量反而会走 rmw_iceoryx_cpp 的 20ms 路径。

## rmw_iceoryx_cpp 测试

还测试了通过 `rmw_iceoryx_cpp` RMW 层使用 iceoryx 的方式（即将 ROS 2 的默认通信层替换为 iceoryx）。

### 安装

`rmw_iceoryx_cpp` 在 ROS 2 Humble 中无 apt 包，需从源码编译：

```bash
cd /home/gaoyuan/image_delay_test
colcon build --symlink-install --packages-select rmw_iceoryx_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

### 使用

```bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_iceoryx_cpp
/opt/ros/humble/bin/iox-roudi -c roudi_config.toml
ros2 launch image_test image_test.launch.py mode:=4 image_pub_frequency:=200
```

### 测试结果

| 方式 | 10Hz | 100Hz | 200Hz |
|------|------|-------|-------|
| mode=4 + 默认DDS (FastDDS) | 2~5ms | 2.5~5ms | 6~15ms |
| mode=4 + rmw_iceoryx_cpp | 2ms | 20~24ms | 20~24ms |
| mode=5 iceoryx直接通信 | 0.65~1.8ms | 0.4~0.8ms | 0.4~0.5ms |

`rmw_iceoryx_cpp` 在高频下（100Hz+）对大图像消息有约 20ms 的固有序列化开销，性能反而不如默认 DDS。直接使用 iceoryx 原生 API（mode=5）可避免此开销。

## 各方式对比总结

| 方式 | 原理 | 优势 | 劣势 |
|------|------|------|------|
| ros2 image_transport | 标准 ROS 2 DDS 传输 | 兼容性好，支持可视化工具 | 延迟较高 |
| ros2 loaned msg + shm_msg | DDS 共享内存 + loaned API | 零拷贝，延迟较低 | 需要配置 DDS 共享内存 |
| shm_video_transmission | Boost.Interprocess 共享内存 | 延迟低，不依赖 DDS | 仅支持 CV_8UC3，需手动管理共享内存 |
| UltraMultiThread | 进程内 shared_ptr 传递 | 延迟极低，无系统调用 | 仅限同一进程内通信，需 ROS 2 Component |
| iceoryx直接通信 | iceoryx 原生 API 共享内存 | 延迟最低的跨进程方案 | 需要 RouDi 守护进程，需自定义内存池配置 |

## 进一步优化方向

当前 mode=5 (0.24-0.52ms) 已接近 CPU 侧 memcpy 的物理极限（5.9MB / 20GB/s ≈ 0.3ms）。进一步降低延迟需要消除 memcpy：

| 方向 | 预计延迟 | 条件 |
|------|----------|------|
| 相机 V4L2 + DMA-BUF 直写共享内存 | <0.1ms | USB/CSI 相机 |
| CUDA IPC 共享 GPU 内存 | 1-5us | NVIDIA GPU |
| DPDK rte_ring + hugepage | <100ns | 独占 CPU 核 |
| Zenoh SHM (rmw_zenoh) | 10-100us | ROS 2 Jazzy+ |
