# Repository Guidelines

## 项目结构与模块组织

本仓库是 ROS 2 Humble 图像传输延迟测试工作区。核心测试节点在 `src/image_test`，其中 `src/image_test/src` 放组件源码，`src/image_test/launch` 放启动文件，`src/image_test/test` 放 C++/Python 测试。AutoAim 共享内存传输已拆到 `src/autoaim_shm_image_transport`，主要由头文件实现，基于 POSIX 共享内存和单槽环形缓冲，mode 7 支持直接写 shm slot 和源 Mat 拷贝两种口径。其他传输实验位于 `src/ros2_shm_msgs`、`src/shm_video_transmission`、`src/UltraMultiThread` 和 `src/rmw_iceoryx`。脚本放在 `tools/`，新的测试结果建议写入 `benchmark_results/`。

## 构建、测试与开发命令

构建前先加载 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
```

编译主要测试包，并限制并行核数：

```bash
colcon build --symlink-install --packages-up-to image_test --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
```

运行重点测试：

```bash
colcon test --packages-select autoaim_shm_image_transport image_test --ctest-args -R "test_autoaim_shm_transport|test_image_pub_utils" --output-on-failure --parallel 1
python3 src/image_test/test/test_timestamp_position.py
```

运行 mode 7 延迟测试示例：

```bash
bash tools/run_image_benchmark.sh 7 8 200 copy_image:=false generate_in_transport_buffer:=true
bash tools/run_image_benchmark.sh 7 8 200 copy_image:=false generate_in_transport_buffer:=false
```

## 代码风格与命名约定

使用 C++17、CMake 和 ament 约定。新增代码应跟随所在文件的缩进、include 顺序和命名风格。公共头文件优先使用 `.hpp`，实现文件使用 `.cpp`。命名空间保持包级隔离，例如 `image_test` 或 `autoaim_shm_image_transport`。共享内存相关代码优先使用 RAII 管理 fd 和 mmap；只有在内存序不直观时再补充简短注释。

## 测试规范

C++ 测试使用 `ament_cmake_gtest`，Python 检查以可执行脚本形式存在。修改传输模式、时间戳位置、launch 逻辑或共享内存布局时，需要同步新增或更新测试。涉及延迟变化时，基准测试结果应记录 mode、持续时间、发布频率和 launch 参数，保证结果可复现。

## 提交与 Pull Request 规范

提交历史以简短中文摘要为主，通常使用具体的动宾结构，例如 `新增 autoaim 共享内存图像传输包`、`添加 mode 6 rclcpp unique_ptr 图像测试`。PR 应说明改动内容、执行过的命令、影响的 mode；若性能有变化，还应附 p50/p95 延迟或 CPU 观察值。需要特殊运行环境时，请注明 RouDi、RMW 实现和 launch 参数。

## 配置注意事项

mode 5 依赖 ROS 2 iceoryx/RouDi，并使用 `roudi_config.toml`。除非结果本身是改动内容的一部分，否则不要提交生成的 `build/`、`install/`、`log/`、`.ros_log/` 或大体积基准测试输出。
