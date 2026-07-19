#!/bin/bash
# F2C 一站式开发工作流
# 用法:
#   ./sync_and_build.sh                      # 仅同步+编译
#   ./sync_and_build.sh --test [scenario]    # 同步+编译+跑单场景（默认 S8）
#   ./sync_and_build.sh --batch              # 同步+编译+8场景批量测试+渲染+拉回结果
#   ./sync_and_build.sh --compare            # 同步+编译+S3/S4/S6 自定义 vs Snake 对比
#   ./sync_and_build.sh --run                # 同步+编译+远程启动 S8 RViz 演示
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 自动定位项目根目录：无论脚本放在 scripts/ 还是项目根，都能找到 src/
if [[ "$SCRIPT_DIR" == */scripts ]]; then
    PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
else
    PROJECT_ROOT="$SCRIPT_DIR"
fi
# ── 用户配置（使用前请修改）──
VM_HOST="dc@192.168.83.129"   # 改为你的 VM 用户名@IP
WS="~/f2c_coverage_planner"    # 改为你的 VM 端工作空间路径
RESULT_BASE="$PROJECT_ROOT/test_results"

# 场景映射
declare -A SCENARIOS=(
    ["S8"]="${WS}/src/yingshi_robot/config/f2c_areas/notched_10m_with_center_hole.yaml"
    ["S1"]="${WS}/src/yingshi_robot/test_polygons/S1_S1_convex_rect.yaml"
    ["S2"]="${WS}/src/yingshi_robot/test_polygons/S2_S2_L_shaped.yaml"
    ["S3"]="${WS}/src/yingshi_robot/test_polygons/S3_S3_with_holes.yaml"
    ["S4"]="${WS}/src/yingshi_robot/test_polygons/S4_S4_narrow_corridor.yaml"
    ["S5"]="${WS}/src/yingshi_robot/test_polygons/S5_S5_irregular.yaml"
    ["S6"]="${WS}/src/yingshi_robot/test_polygons/S6_S6_multi_region.yaml"
)

# ── Step 0: 检查 SSH 连接 ──
if ! ssh -o ConnectTimeout=3 -o BatchMode=yes "$VM_HOST" "echo ok" >/dev/null 2>&1; then
    echo "❌ 无法连接到 VM ($VM_HOST)"
    echo ""
    echo "请确认："
    echo "  1. VM 已开机且网络正常"
    echo "  2. 已配置 SSH 免密登录（详见 SETUP_GUIDE.md §4）"
    echo "  3. VM_HOST 和用户名正确（编辑本脚本第 18-19 行）"
    exit 1
fi

# ── Step 1: 同步源码 ──
echo "📤 Syncing sources..."
# 同步所有源码 + 模块文件 + 脚本
scp -q "$PROJECT_ROOT/src/yingshi_robot/src/"*.cpp "${VM_HOST}:${WS}/src/yingshi_robot/src/"
scp -q "$PROJECT_ROOT/src/yingshi_robot/src/"*.hpp "${VM_HOST}:${WS}/src/yingshi_robot/src/"
scp -q "$PROJECT_ROOT/src/yingshi_robot/include/yingshi_robot/"*.hpp "${VM_HOST}:${WS}/src/yingshi_robot/include/yingshi_robot/"
scp -q "$PROJECT_ROOT/src/yingshi_robot/CMakeLists.txt" "${VM_HOST}:${WS}/src/yingshi_robot/"
scp -q "$PROJECT_ROOT/scripts/"*.sh "${VM_HOST}:${WS}/scripts/" 2>/dev/null || true
scp -q "$PROJECT_ROOT/scripts/"*.py "${VM_HOST}:${WS}/scripts/" 2>/dev/null || true
echo "   Done."

# ── Step 2: 编译 ──
echo "🔨 Building..."
BUILD_OUT=$(ssh "${VM_HOST}" "cd ${WS} && source /opt/ros/humble/setup.bash && colcon build --packages-select yingshi_robot 2>&1")
if echo "$BUILD_OUT" | grep -qE "error:|fatal error"; then
    echo "❌ Build failed:"
    echo "$BUILD_OUT" | grep -E "error:|fatal error" | head -10
    exit 1
fi
echo "   $(echo "$BUILD_OUT" | grep 'Summary' || echo 'Done')"

# ── 解析命令行 ──
MODE="${1:-}"
SCENARIO="${2:-S8}"

# ── Mode: --test ──
if [[ "$MODE" == "--test" ]]; then
    YAML="${SCENARIOS[$SCENARIO]:-${SCENARIOS[S8]}}"
    echo ""
    echo "🧪 Testing: ${SCENARIO}..."

    RESULT=$(ssh "${VM_HOST}" "
        source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash && \
        export LD_LIBRARY_PATH=${WS}/install/fields2cover/lib:${WS}/install/lib:${WS}/src/Fields2Cover/third_party/ortools-src/lib:\$LD_LIBRARY_PATH && \
        LOG=/tmp/f2c_test_\$(date +%H%M%S).log && \
        ros2 run yingshi_robot polygon_planner_node --ros-args \
          -p use_planner_core:=true \
          -p robot_width:=0.75 -p coverage_width:=0.90 \
          -p mid_hl_width_ratio:=0.20 -p no_hl_width_ratio:=0.0 \
          -p use_sweep_decomp:=true -p merge_angle_threshold:=60.0 \
          -p swath_order_type:=boustrophedon \
          -p boundary_type:=closed -p turn_planner_type:=direct \
          -p use_optimized_planner:=true -p decomposition_enabled:=true \
          -p path_simplify_enabled:=true -p swath_angle_optimization:=true \
          -p filter_tiny_cells:=true -p swath_endpoint_shrink_distance:=0.03 \
          -p min_swath_length:=0.5 -p path_simplify_tolerance:=0.05 \
          -p swath_overlap_ratio:=0.03 -p eval_enable_report:=true \
          -p eval_use_grid_method:=true -p eval_grid_resolution:=0.1 \
          > \$LOG 2>&1 & \
        PID=\$! && sleep 4 && \
        python3 - \"${YAML}\" << 'PYEOF'
import sys, time, yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon
rclpy.init()
n = Node('qt')
with open(sys.argv[1]) as f: a = yaml.safe_load(f)
pp = n.create_publisher(Polygon, '/input_polygon_1', 10)
hp = n.create_publisher(Polygon, '/input_polygon_1_holes', 10)
pm = Polygon()
for p in a['polygon']: pm.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))
hm = Polygon()
for hi, h in enumerate(a.get('holes', [])):
    if hi > 0: hm.points.append(Point32(x=1e10, y=0.0, z=0.0))
    for p in h: hm.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))
for _ in range(3):
    hp.publish(hm); rclpy.spin_once(n, timeout_sec=0.1); time.sleep(0.2)
    pp.publish(pm); time.sleep(2.0); rclpy.spin_once(n, timeout_sec=0.1)
for _ in range(90):
    rclpy.spin_once(n, timeout_sec=0.5); time.sleep(0.5)
n.destroy_node(); rclpy.shutdown()
PYEOF
        wait \$PID 2>/dev/null
        echo '===RESULTS==='
        grep -E '覆盖率|综合得分|转弯次数|重叠率|路径总长|未覆盖面积|有效工作比|hole.aware|RemArea fill' \$LOG | head -20
        echo \"LOG=\$LOG\"
    " 2>&1)

    echo "$RESULT"

# ── Mode: --batch ──
elif [[ "$MODE" == "--batch" ]]; then
    echo ""
    echo "📊 Batch testing 6 scenarios..."
    TIMESTAMP=$(date +%m%d_%H%M)
    VM_RESULT="$WS/test_results/batch_${TIMESTAMP}"
    mkdir -p "$RESULT_BASE/batch_${TIMESTAMP}"

    ssh "${VM_HOST}" "
        source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash && \
        export LD_LIBRARY_PATH=${WS}/install/fields2cover/lib:${WS}/install/lib:${WS}/src/Fields2Cover/third_party/ortools-src/lib:\$LD_LIBRARY_PATH && \
        bash ${WS}/scripts/batch_test_v2.sh ${VM_RESULT} 2>&1
    " 2>&1 | tee /tmp/f2c_batch.log

    echo ""
    echo "📥 Copying results..."
    scp -q "${VM_HOST}:${VM_RESULT}/*.png" "$RESULT_BASE/batch_${TIMESTAMP}/" 2>/dev/null || true
    scp -q "${VM_HOST}:${VM_RESULT}/*.json" "$RESULT_BASE/batch_${TIMESTAMP}/" 2>/dev/null || true
    scp -q "${VM_HOST}:${VM_RESULT}/*.txt" "$RESULT_BASE/batch_${TIMESTAMP}/" 2>/dev/null || true
    echo "   → $RESULT_BASE/batch_${TIMESTAMP}/"

# ── Mode: --compare ──
elif [[ "$MODE" == "--compare" ]]; then
    echo ""
    echo "🔬 Comparing Custom vs Snake (S3, S4, S6)..."
    TIMESTAMP=$(date +%m%d_%H%M)
    mkdir -p "$RESULT_BASE/compare_${TIMESTAMP}"

    ssh "${VM_HOST}" "
        source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash && \
        export LD_LIBRARY_PATH=${WS}/install/fields2cover/lib:${WS}/install/lib:${WS}/src/Fields2Cover/third_party/ortools-src/lib:\$LD_LIBRARY_PATH && \
        bash ${WS}/scripts/compare_snake_vs_custom.sh 2>&1
    " 2>&1

    echo ""
    echo "📥 Copying comparison images..."
    # 找最新 compare 目录
    LATEST_COMPARE=$(ssh "${VM_HOST}" "ls -td ${WS}/test_results/compare_snake_* | head -1")
    if [ -n "$LATEST_COMPARE" ]; then
        scp -q "${VM_HOST}:${LATEST_COMPARE}/*.png" "$RESULT_BASE/compare_${TIMESTAMP}/" 2>/dev/null || true
    fi
    echo "   → $RESULT_BASE/compare_${TIMESTAMP}/"

# ── Mode: --run ──
elif [[ "$MODE" == "--run" ]]; then
    echo ""
    echo "🚀 Launching S8 scenario on VM..."
    echo "   (Press Ctrl-C in VM terminal to stop)"
    ssh -t "${VM_HOST}" "
        source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash && \
        export LD_LIBRARY_PATH=${WS}/install/fields2cover/lib:${WS}/install/lib:${WS}/src/Fields2Cover/third_party/ortools-src/lib:\$LD_LIBRARY_PATH && \
        bash ${WS}/install/yingshi_robot/share/yingshi_robot/scripts/run_f2c_optimized.sh 2>&1
    "

# ── Mode: default (sync+build only) ──
else
    echo ""
    echo "✅ Sync + build done."
    echo ""
    echo "Usage:"
    echo "  ./sync_and_build.sh --test [S1-S8]   # 单场景快速测试"
    echo "  ./sync_and_build.sh --batch                    # 8场景批量测试+渲染"
    echo "  ./sync_and_build.sh --compare                  # 自定义 vs Snake 对比"
    echo "  ./sync_and_build.sh --run                      # 启动 S8 RViz"
fi
