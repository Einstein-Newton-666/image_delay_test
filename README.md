# ros2图像传输延迟测试

测试对象：ros2 image_transport，shm_video_transmission，UltraMultiThread

测试平台：AMD Ryzen 7 5800H

测试方式：在ros2 component 中以200hz发布1920*1024大小的图像，查看图像传输延迟和cpu占用率 （未关闭ros info）

测试结果：

| 图像传输方式                         | 传输延迟    | cpu占用率 |
| :----------------------------------- | ----------- | --------- |
| ros2 image_transport（queue_size=1） | 2~11ms      | 50%       |
| shm_video_transmission               | 0.4~1.8ms   | 20%       |
| UltraMultiThread                     | 0.04~0.09ms | 15%       |

