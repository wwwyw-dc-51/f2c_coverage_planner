#!/usr/bin/env bash
# ============================================================
# F2C WSL2 全量批测包装脚本
# 用法（在 WSL Ubuntu 终端中运行）:
#   source scripts/wsl_batch.sh
#   或者直接跑:
#   bash scripts/wsl_batch.sh
# ============================================================
set -eo pipefail

echo "=== F2C WSL2 环境初始化 ==="

# 1. 清理 Windows PATH 污染
export PATH=/opt/ros/humble/bin:/usr/bin:/bin:/usr/local/bin:/usr/sbin:$HOME/.local/bin

# 2. ROS2 环境
export AMENT_PREFIX_PATH=/opt/ros/humble
export PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
export LD_LIBRARY_PATH=/opt/ros/humble/lib

# 3. 项目环境
WS="$HOME/f2c_coverage_planner"
export AMENT_PREFIX_PATH=/opt/ros/humble:$WS/install
export LD_LIBRARY_PATH=$WS/install/lib:$WS/install/fields2cover/lib:$LD_LIBRARY_PATH
export DISPLAY="${DISPLAY:-:0}"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash" 2>/dev/null || true

cd "$WS"

echo "ROS_DISTRO=$ROS_DISTRO"
echo "LD_LIBRARY_PATH 已配置"
echo ""

# 4. 运行批测
exec bash scripts/batch_test_v2.sh "$@"
