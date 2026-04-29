# usb_cam [![ROS 2 CI](https://github.com/ros-drivers/usb_cam/actions/workflows/build_test.yml/badge.svg)](https://github.com/ros-drivers/usb_cam/actions/workflows/build_test.yml)

## 基于 V4L 的 ROS 2 USB 相机驱动

这个包面向 V4L 设备，而不只是通用 UVC 设备。

如果你需要 ROS 1 文档，请参考 [ROS wiki](http://ros.org/wiki/usb_cam)。

## 支持的 ROS 2 发行版与平台

所有官方支持的 Linux 发行版及其对应的 ROS 2 版本都应可使用。如果你在这些平台上遇到问题，欢迎提交 issue。

Windows：待定 / 未测试 / 未验证  
macOS：待定 / 未测试 / 未验证

如果你愿意在 macOS 或 Windows 上尝试并推动它可用，也欢迎提交 issue 记录过程。若验证成功，我们可以把对应说明补充到这里。

## 快速开始

如果你已经安装了受支持的 ROS 2 发行版，可以先尝试直接安装二进制版本：

```shell
sudo apt-get install ros-<ros2-distro>-usb-cam
```

目前这个包通常可以在所有活跃支持的 ROS 2 发行版中直接通过二进制方式安装。

如果你无法直接安装二进制包，请继续参考下面的源码编译步骤。

## 从源码构建

把源码克隆或下载到你的工作区中：

```shell
cd /path/to/colcon_ws/src
git clone https://github.com/ros-drivers/usb_cam.git
```

你也可以直接在 GitHub 页面点击绿色的 “Download zip” 按钮下载源码压缩包。

下载完成后，确认已经 source 过 ROS 2 underlay，然后安装依赖：

```shell
cd /path/to/colcon_ws
rosdep install --from-paths src --ignore-src -y
```

依赖安装完成后，就可以开始编译 `usb_cam`：

```shell
cd /path/to/colcon_ws
colcon build
source /path/to/colcon_ws/install/setup.bash
```

编译成功后，记得 source 新生成的环境。

完成后，你可以按照下一节的方式运行这个包。

## 运行

`usb_cam_node` 可以直接使用默认参数运行，也可以通过命令行或参数文件传入自定义参数。

仓库中提供了示例参数文件 [config/params_1.yaml](config/params_1.yaml) 作为起点，你可以按自己的需求修改。

同时也提供了一个 launch 文件，用于启动 `usb_cam_node_exe`，并附带一个额外的图像显示节点。

下面给出几种常见启动方式：

**注意：下面的命令只需要选择一种执行即可**

```shell
# 使用默认配置直接运行
ros2 run usb_cam usb_cam_node_exe

# 使用 yaml 参数文件运行
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /path/to/colcon_ws/src/usb_cam/config/params_1.yaml

# 使用 launch 文件启动
ros2 launch usb_cam camera.launch.py
```

## 同时启动多个 usb_cam 节点

如果要同时启动多个节点，只需要为每个节点设置不同的命名空间并使用不同的参数文件：

```shell
ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:=/usb_cam_0 --params-file /path/to/usb_cam/config/params_1.yaml
ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:=/usb_cam_1 --params-file /path/to/usb_cam/config/params_2.yaml
```

## 支持的格式

### 设备支持的格式

要查看当前连接设备支持哪些格式，可以运行 `usb_cam_node` 并观察控制台输出。

示例输出如下：

```log
This devices supproted formats:
       Motion-JPEG: 1280 x 720 (30 Hz)
       Motion-JPEG: 960 x 540 (30 Hz)
       Motion-JPEG: 848 x 480 (30 Hz)
       Motion-JPEG: 640 x 480 (30 Hz)
       Motion-JPEG: 640 x 360 (30 Hz)
       YUYV 4:2:2: 640 x 480 (30 Hz)
       YUYV 4:2:2: 1280 x 720 (10 Hz)
       YUYV 4:2:2: 640 x 360 (30 Hz)
       YUYV 4:2:2: 424 x 240 (30 Hz)
       YUYV 4:2:2: 320 x 240 (30 Hz)
       YUYV 4:2:2: 320 x 180 (30 Hz)
       YUYV 4:2:2: 160 x 120 (30 Hz)
```

### 驱动支持的格式

驱动本身也有自己支持的格式列表，详情可以查看 [include/usb_cam/formats/](include/usb_cam/formats/) 下的源码。

在确认了 [设备支持的格式](#设备支持的格式) 后，可以通过参数文件中的 `pixel_format` 参数指定实际要使用的格式。

如果你想查看驱动当前支持的全部格式，可以运行：

```shell
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:="test"
```

这里的 `"test"` 也可以换成任意一个不受支持的像素格式字符串。驱动会在报错前打印出自己支持的格式列表。

如果你需要更多格式或转换支持，也欢迎提交贡献。

### 支持的 IO 方法

当前驱动支持以下三种 IO 方法：

1. `read`：在用户态与内核态之间复制视频帧数据
2. `mmap`：使用由内核空间分配的内存映射缓冲区
3. `userptr`：使用由用户空间分配的内存缓冲区

如果你想进一步了解这些 IO 方式的区别，可以参考这篇文章：[A V4L2 driver overview](https://lwn.net/Articles/240667/)

### 低延迟采集参数

对于双目、SLAM 或更关注“最新帧”而不是“严格匀速发布”的场景，节点还提供了几项低延迟相关参数：

1. `low_latency_mode`：设为 `true` 后，每次更新会尽量排空驱动队列，并在拿到最新帧后立即发布，而不是再经过第二个独立的发布定时器
2. `buffer_count`：请求更小的 V4L2 缓冲队列，例如设置为 `2`，在某些相机上可以减小启动时和稳态运行时的帧积压
3. `startup_frame_drop_count`：在启动后主动丢弃前几帧，避免流水线一开始就处理到已经滞留在缓冲区里的旧图像

示例参数文件 [config/params_1.yaml](config/params_1.yaml) 和 [config/params_2.yaml](config/params_2.yaml) 已默认启用了这一模式，并设置了 `buffer_count=2`、`startup_frame_drop_count=4`。

## 压缩

感谢 [ros2_v4l2_camera](https://gitlab.com/boldhearts/ros2_v4l2_camera#usage-1) 项目及其文档在这部分提供的参考。

`usb_cam` 默认支持压缩图像发布，因为它使用了 `image_transport`。只要你的系统中已经安装了 `image_transport_plugins`，`usb_cam` 通常就会自动发布 `compressed` 主题。

不过，目前 `rviz2` 和 `show_image.py` 还不能直接显示压缩图像，因此你可能需要在下游把压缩图像重新发布为未压缩图像：

```shell
ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=image_raw/compressed --remap out:=image_raw/uncompressed
```

## 测试

运行基础单元测试：

```shell
colcon build --packages-select usb_cam
colcon test --pacakges-select usb_cam
```

### 集成测试

运行集成测试：

```shell
colcon build --packages-select usb_cam --cmake-args -DINTEGRATION_TESTS=1
colcon test --pacakges-select usb_cam
```

### Address Sanitizer 与 Leak Sanitizer

`CMakeLists.txt` 中已经集成了用于辅助检查内存泄漏和地址问题的编译开关。

如果要启用这些检查，请在构建时传入 `SANITIZE=1`：

```shell
colcon build --packages-select usb_cam --cmake-args -DSANITIZE=1
```

构建完成后，可以直接运行节点可执行文件，并按需传入 `ASAN_OPTIONS`：

```shell
ASAN_OPTIONS=new_delete_type_mismatch=0 ./install/usb_cam/lib/usb_cam/usb_cam_node_exe
```

使用 `Ctrl+C` 关闭节点后，sanitizer 会输出检测到的内存问题。

默认情况下这项功能是关闭的，因为启用 sanitizer 会增加可执行文件体积，并降低运行性能。

## 文档

可以在 ROS wiki 上找到 [Doxygen](http://docs.ros.org/indigo/api/usb_cam/html/) 文档。

### 许可证

`usb_cam` 使用 BSD 许可证发布。完整条款请查看 [LICENSE](LICENSE) 文件。

### 作者

完整贡献者列表请查看 [AUTHORS](AUTHORS.md) 文件。
