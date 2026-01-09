# dev_images

## 简介

- 该工具集用于从 ROS2 的 bag 包中解析并保存用于开发与测试的相机图片（PNG/JPEG）和点云（PCD）。主要面向需要提取样例数据进行离线分析或可视化的开发场景。

## 用法

# 修改源码中接收话题为你系统中实际的 topic（例如在 C++ 示例 `save_compressed_image.cpp` 中）
# 在工作区根目录构建并加载安装环境

```bash
# 在工作区根目录执行（将 <package_name> 替换为实际包名）
colcon build --packages-select <package_name>
# 构建完成后 source 安装环境
source install/setup.bash
```

# 运行保存程序（示例）

```bash
ros2 run image_saver_cpp save_compressed_image

ros2 bag play <your_bag>.db3/mcap
```

## 功能

- 从 ROS/ROS2 bag 中提取压缩或未压缩的相机帧并保存为 PNG/JPEG。
- 从 bag 中提取点云并保存为 PCD（若示例代码包含点云处理）。
- 示例代码和脚本可作为快速提取样本数据用于算法调试或可视化。

## 依赖

- ROS2（取决于示例代码使用的 API）。
- PCL（如果要处理或保存点云文件）。
- OpenCV（如果示例代码中包含图像解压或处理）。

## 注意事项

- 在修改或运行前，请先确认代码中订阅的 topic 名称与 bag 文件中一致。
- 如果使用压缩图像（CompressedImage），确保示例代码使用正确的解压流程。
- 本仓库仅提供示例与工具，真实项目中请根据需要加强错误处理与路径管理。

## 示例数据与目录

- 仓库中包含若干示例数据和输出目录（见仓库结构中的 `data/` 与 `output/`），可用来快速验证提取逻辑。