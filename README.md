# ros2图像传输延迟测试

对比测试多种图像传输方式和 RMW 配置的延迟和 CPU 占用率。

测试平台：AMD Ryzen 7 5800H | WSL2 Linux 6.6.114.1 | 16 vCPU | /dev/shm 4.9G | 图像：1920x1024 CV_8UC3 (5.9MB) | 频率：200Hz

延迟口径：发布端可用 `cv::Mat` 到订阅端可用 `cv::Mat` 的端到端延迟。`generate_in_transport_buffer:=true` 时，mode=4/5/6 的发布端 `cv::Mat` 直接包装 loaned payload / iceoryx chunk / ROS Image data，延迟不包含图像填充；`generate_in_transport_buffer:=false` 时，先生成普通 `cv::Mat`，再拷贝到 loaned payload / iceoryx chunk / ROS Image data，延迟包含 loan/分配、拷贝、publish/take。mode=1/2/3/7 仍从普通 `cv::Mat` 开始。

CPU 口径：`component_container` 进程级平均 CPU，占 all-core / one-core；包含图像填充、发布、订阅、回调和日志开销，不是仅传输函数局部 CPU。

## 测试结果汇总（2026-06-06 统一重测，mode=6 于 2026-06-07 补测，mode=7 于 2026-06-11 补测）

所有结果均使用当前代码、200Hz、8s、`queue_size:=1` 重测；旧口径、补充结果、无效配置、fallback 和官方不支持 loaned API 的 mode=4 组合不再单独列出。mode=4 的 `loan` 列来自运行日志中的 `mode 4 loaned image messages`。

| 传输方式 | RMW / 配置 | mode | intra | loan | 延迟 p50 / p95 | CPU 均值 all / one-core | 备注 |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| UltraMultiThread | - | 3 | - | - | **0.076 / 0.165ms** | 2.22% / 35.51% | 源 Mat 可用后移交 ImagePack |
| iceoryx 直接通信 | - | 5 | - | - | **0.110 / 0.188ms** | 1.83% / 29.32% | `generate_in_transport_buffer=true`，直接在 iceoryx chunk 生成图像 |
| raw rclcpp Image | FastDDS | 6 | True | - | **0.121 / 0.222ms** | 2.66% / 42.59% | `generate_in_transport_buffer=true`，同进程 `unique_ptr` intra-process，1605 samples |
| iceoryx 直接通信 | - | 5 | - | - | **0.760 / 1.044ms** | 2.48% / 39.73% | `generate_in_transport_buffer=false`，源 Mat copy 到 iceoryx chunk |
| auto_aim POSIX shm ring | - | 7 | False | - | **1.414 / 1.614ms** | 2.95% / 47.17% | `copy_image=false`，源 Mat memcpy 到单槽 shm，订阅端零拷贝包装 |
| shm_video_transmission | - | 2 | - | - | **1.588 / 3.272ms** | 3.99% / 63.75% | 源 Mat memcpy 到共享内存 |
| raw rclcpp Image | FastDDS | 6 | True | - | 1.707 / 2.082ms | 3.31% / 52.98% | `generate_in_transport_buffer=false`，普通源 Mat copy 到 ROS Image data，1458 samples |
| auto_aim POSIX shm ring | - | 7 | False | - | 2.081 / 2.743ms | 3.93% / 62.85% | `copy_image=true`，订阅端 clone，贴近 auto_aim 原始取图路径 |
| loaned API + shm_msg | FastDDS + `shm_fastdds.xml` | 4 | False | enabled | 2.301 / 3.135ms | 4.10% / 65.53% | `generate_in_transport_buffer=true`，直接在 payload 生成图像 |
| loaned API + shm_msg | FastDDS + `shm_fastdds.xml` | 4 | False | enabled | 4.141 / 6.487ms | 5.97% / 95.45% | `generate_in_transport_buffer=false`，源 Mat copy 到 payload |
| ros2 image_transport | FastDDS | 1 | False | - | 4.457 / 9.706ms | 6.03% / 96.53% | 跨进程 DDS |
| ros2 image_transport | rmw_zenoh_shm | 1 | True | - | 5.263 / 10.034ms | 6.01% / 96.20% | 单进程 intra |
| ros2 image_transport | rmw_zenoh | 1 | True | - | 6.324 / 13.973ms | 6.07% / 97.14% | 单进程 intra |
| ros2 image_transport | FastDDS | 1 | True | - | 6.607 / 13.759ms | 6.21% / 99.31% | 单进程 intra |
| ros2 image_transport | rmw_iceoryx_cpp | 1 | False | - | 73.584 / 139.597ms | 5.42% / 86.80% | 仅 69 samples，作为 ROS RMW 不稳定 |

### 关键结论

- **mode=4 只保留真正 loaned API 结果**：FastDDS 必须加载 `src/ros2_shm_msgs/config/shm_fastdds.xml` 并设置 `RMW_FASTRTPS_USE_QOS_FROM_XML=1`，发布端 `can_loan_messages()` 为 true。
- **Zenoh SHM 不是 ROS 2 loaned API**：官方 `rmw_zenoh` 不支持 `rmw_borrow_loaned_message()`，因此 Zenoh 的 mode=4 结果不再作为 loaned API 测试项列入表格。
- **rmw_iceoryx_cpp 作为 ROS RMW 表现较差**：mode=1 普通 `image_transport` 仅收到 69 samples，p50 约 73.6ms；其 mode=4 结果属于不支持/不稳定的 loaned API 测试项，不列入表格。
- **共享内存 copy 成本可直接对比**：mode=5 direct/copy p50 为 0.110ms / 0.760ms；mode=4 direct/copy p50 为 2.301ms / 4.141ms。
- **auto_aim 共享内存取图**：mode=7 使用 POSIX `shm_open` + `mmap` + futex 的单槽 ring；零拷贝包装 p50 1.414ms，订阅端 clone p50 2.081ms。
- **mode=7 的实现已拆成独立包**：共享内存 ring 头文件位于 `src/autoaim_shm_image_transport/`，`image_test` 只作为 benchmark 使用方。
- **最快 IPC/传输路径**：mode=3、mode=5 direct、mode=6 direct、mode=5 copy、mode=7 no-copy、mode=2、mode=6 copy 的 p50 分别为 0.076ms、0.110ms、0.121ms、0.760ms、1.414ms、1.588ms、1.707ms。
- **mode=6 是同进程 unique_ptr 基线**：不经过 `image_transport`，也不是共享内存 IPC；`generate_in_transport_buffer=true` 测相机直接写 ROS Image data，`false` 测普通相机 Mat copy 到 ROS Image data。
- **intra-process 只适合同进程组件**：mode=1 intra-process p50 约 5.26-6.61ms，不是跨进程 IPC 结果。
- **WSL2 结果只代表当前环境**：线程调度、优先级设置和共享内存实现都可能与原生 Linux 不同。

## 运行方式

```bash
# 构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# mode=1~4（默认 DDS）
ros2 launch image_test image_test.launch.py mode:=1 image_pub_frequency:=200

# mode=6（同进程 rclcpp unique_ptr，不经过 image_transport）
ros2 launch image_test image_test.launch.py mode:=6 use_intra_process_comms:=true image_pub_frequency:=200 generate_in_transport_buffer:=true
ros2 launch image_test image_test.launch.py mode:=6 use_intra_process_comms:=true image_pub_frequency:=200 generate_in_transport_buffer:=false

# mode=4/5 对比图像生成位置
# true：直接在 loaned payload / iceoryx chunk 中生成图像
# false：先生成普通 cv::Mat，再 copy 到 loaned payload / iceoryx chunk
ros2 launch image_test image_test.launch.py mode:=4 generate_in_transport_buffer:=true
ros2 launch image_test image_test.launch.py mode:=4 generate_in_transport_buffer:=false

# mode=5（需要先启动 RouDi）
/opt/ros/humble/bin/iox-roudi -c roudi_config.toml &
unset RMW_IMPLEMENTATION
ros2 launch image_test image_test.launch.py mode:=5 image_pub_frequency:=200 generate_in_transport_buffer:=true

# mode=7（auto_aim POSIX shm ring，launch 会启动发布/订阅两个 component_container 进程）
ros2 launch image_test image_test.launch.py mode:=7 image_pub_frequency:=200 copy_image:=false
ros2 launch image_test image_test.launch.py mode:=7 image_pub_frequency:=200 copy_image:=true
```

## RMW 切换

```bash
# FastDDS（默认配置，不开启 loaned/DataSharing）
unset RMW_IMPLEMENTATION

# FastDDS loaned/DataSharing（ros2_shm_msgs 推荐配置）
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/gaoyuan/image_delay_test/src/ros2_shm_msgs/config/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
unset ROS_DISABLE_LOANED_MESSAGES

# rmw_zenoh + 共享内存
sudo apt install ros-humble-rmw-zenoh-cpp
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
ros2 daemon stop
ros2 run rmw_zenoh_cpp rmw_zenohd &

# rmw_iceoryx_cpp（从源码编译，不推荐）
colcon build --symlink-install --packages-select rmw_iceoryx_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
export RMW_IMPLEMENTATION=rmw_iceoryx_cpp
# 需要先启动 RouDi
/opt/ros/humble/bin/iox-roudi -c /home/gaoyuan/image_delay_test/roudi_config.toml &
```

## 优化措施

| 优化 | 适用模式 | 效果 |
|------|---------|------|
| 预分配 cv::Mat（避免每帧 malloc） | 1/2/3 | 降低 0.3-0.8ms |
| 修复 toCvShare 编码不匹配 | 1 | 降低 1.5-3.0ms |
| 检查 `can_loan_messages()` | 4 | 确认结果是真正 loaned buffer |
| move 语义避免 ImagePack 拷贝 | 3 | 降低 1.0-2.0ms |
| 零拷贝 wrapper（直接写入目标 buffer） | 4/5 | 消除普通 Mat 到共享内存/loaned payload 的中间拷贝 |
| 目标 `cv::Mat` 可用后打时间戳 | 4/5 | 统计目标 buffer 可用到订阅端可用的路径，排除图像填充 |
| 源 `cv::Mat` 可用后打时间戳 | 1/2/3/7，4/5 copy | 统计源 Mat 到订阅端可用路径，包含消息创建、拷贝和 publish/take |
| -O3 -march=native 编译优化 | 全部 | 降低 0.1-0.3ms |

## 为什么 ROS 2 共享内存延迟大

```
mode=4 FastDDS loaned: shm_fastdds.xml + RMW_FASTRTPS_USE_QOS_FROM_XML=1 → rclcpp::LoanedMessage 可用  (~2ms)
mode=5: 应用 → iceoryx → 共享内存 → iceoryx → 应用  (~0.1ms)
```

## 进一步优化方向

| 方向 | 预计延迟 | 条件 |
|------|----------|------|
| 相机 V4L2 + DMA-BUF 直写共享内存 | <0.1ms | USB/CSI 相机 |
| CUDA IPC 共享 GPU 内存 | 1-5us | NVIDIA GPU |
| DPDK rte_ring + hugepage | <100ns | 独占 CPU 核 |
