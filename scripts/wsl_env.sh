#!/bin/bash
# F2C WSL2 环境设置 — 绕过 Windows PATH 括号问题
# 用法: source ~/f2c_coverage_planner/scripts/wsl_env.sh

export PATH=/opt/ros/humble/bin:/usr/bin:/bin:/usr/local/bin:/usr/sbin
export AMENT_PREFIX_PATH=/opt/ros/humble:$HOME/f2c_coverage_planner/install
export PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$HOME/f2c_coverage_planner/install/fields2cover/lib:$HOME/f2c_coverage_planner/src/Fields2Cover/third_party/ortools-src/lib
export DISPLAY=:0
cd $HOME/f2c_coverage_planner
echo "F2C 环境就绪 | ROS_DISTRO=humble | DISPLAY=$DISPLAY"
