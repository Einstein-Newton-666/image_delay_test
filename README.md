# ros2图像传输延迟测试

对比测试多种图像传输方式和 RMW 配置的延迟和 CPU 占用率。

测试平台：AMD Ryzen 7 5800H | 图像：1920x1024 CV_8UC3 (5.9MB) | 频率：200Hz

## 全部测试结果汇总

| 传输方式 | RMW | mode | intra | 延迟 | CPU |
| :--- | :--- | :---: | :---: | :---: | :---: |
| UltraMultiThread | - | 3 | - | **0.04~0.09ms** | 0.3% |
| iceoryx 直接通信 | - | 5 | - | **0.2~0.4ms** | 8% |
| shm_video_transmission | - | 2 | - | 1.3~2.0ms | 9% |
| loaned msg + shm_msg | FastDDS | 4 | False | 1.5~2.5ms | 8% |
| ros2 image_transport | FastDDS | 1 | False | 2.5~3.5ms | 16% |
| loaned msg + shm_msg | rmw_zenoh_shm | 4 | False | 2.3~2.9ms | - |
| ros2 image_transport | FastDDS | 1 | **True** | 4.0~4.6ms | - |
| ros2 image_transport | rmw_zenoh_shm | 1 | **True** | 4.1~4.7ms | - |
| ros2 image_transport | rmw_zenoh | 1 | **True** | 4.3~5.0ms | - |
| loaned msg + shm_msg | rmw_zenoh | 4 | False | 10~11ms | - |
| loaned msg + shm_msg | rmw_iceoryx_cpp | 4 | False | 20~24ms / 崩溃 | - |

### 关键结论

- **最快跨进程方案**：mode=5 iceoryx 直接通信 (0.2-0.4ms)，绕过整个 ROS 2 中间件栈
- **最快 ROS 2 RMW**：FastDDS 默认 (1.5-2.5ms)
- **intra-process 无帮助**：启用后所有 RMW 延迟趋同 ~4-5ms，且与 loaned msg 不兼容
- **rmw_iceoryx_cpp 不可用**：大图像消息延迟 20ms+，且会崩溃
- **rmw_zenoh_shm 接近 FastDDS**：2.3-2.9ms，但需要额外启动 Zenoh router

## 运行方式

```bash
# 构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# mode=1~4（默认 DDS）
ros2 launch image_test image_test.launch.py mode:=1 image_pub_frequency:=200

# mode=5（需要先启动 RouDi）
/opt/ros/humble/bin/iox-roudi -c roudi_config.toml &
unset RMW_IMPLEMENTATION
ros2 launch image_test image_test.launch.py mode:=5 image_pub_frequency:=200
```

## RMW 切换

```bash
# FastDDS（默认，无需设置）
unset RMW_IMPLEMENTATION

# rmw_zenoh + 共享内存
sudo apt install ros-humble-rmw-zenoh-cpp
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/link/shared_memory/enabled=true'
ros2 daemon stop
ros2 run rmw_zenoh_cpp rmw_zenohd &

# rmw_iceoryx_cpp（从源码编译，不推荐）
colcon build --symlink-install --packages-select rmw_iceoryx_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
export RMW_IMPLEMENTATION=rmw_iceoryx_cpp
```

## 优化措施

| 优化 | 适用模式 | 效果 |
|------|---------|------|
| 预分配 cv::Mat（避免每帧 malloc） | 1/2/3 | 降低 0.3-0.8ms |
| 修复 toCvShare 编码不匹配 | 1 | 降低 1.5-3.0ms |
| 直接写入 loaned 消息 buffer | 4 | 降低 1.0-2.0ms |
| move 语义避免 ImagePack 拷贝 | 3 | 降低 1.0-2.0ms |
| 零拷贝 wrapper（直接写入共享内存） | 5 | 消除中间 buffer |
| -O3 -march=native 编译优化 | 全部 | 降低 0.1-0.3ms |

## 为什么 ROS 2 共享内存延迟大

```
mode=4: 应用 → rclcpp → rmw → DDS → 序列化 → 共享内存 → 反序列化 → DDS → rmw → rclcpp → 应用  (2-5ms)
mode=5: 应用 → iceoryx → 共享内存 → iceoryx → 应用  (0.2-0.4ms)
```

## 进一步优化方向

| 方向 | 预计延迟 | 条件 |
|------|----------|------|
| 相机 V4L2 + DMA-BUF 直写共享内存 | <0.1ms | USB/CSI 相机 |
| CUDA IPC 共享 GPU 内存 | 1-5us | NVIDIA GPU |
| DPDK rte_ring + hugepage | <100ns | 独占 CPU 核 |
