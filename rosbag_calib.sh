#!/bin/bash
# =====================================================
# 一键启动录制所有传感器topic脚本
# Author: mengchengzhen
# Date: 2025-11-05
# =====================================================

set -e  # 出错自动退出
set -o pipefail

# === 环境初始化 ===
echo ">>> Start transfer image/pcd ..."
rm -rf /workspace/dev_images/saved*
source /workspace/dev_images/install/setup.bash
ros2 run image_saver_cpp save_compressed_image