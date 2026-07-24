# F2C 全覆盖路径规划 — 配置与使用指南

> **版本**: v10.0 十全十美 (2026-07-24)
> **环境**: Windows 11 宿主机 + Ubuntu 22.04 (VMware 虚拟机 / WSL2)
> **测试**: 21/21 GATE PASS (WSL2 批测得分全面超越 VM 基线)

---

## 目录

- [1. 环境要求](#1-环境要求)
- [2. 项目结构](#2-项目结构)
- [3. VM 端部署](#3-vm-端部署在-vm-上操作)
- [4. Windows 端配置 SSH](#4-windows-端配置-ssh在本机上操作)
- [5. 日常使用](#5-日常使用)
- [6. 测试场景说明](#6-测试场景说明)
- [7. 常见问题](#7-常见问题)

---

## 1. 环境要求

### Windows 宿主机
| 软件 | 用途 | 下载 |
|------|------|------|
| Git Bash | 运行脚本、SSH/SCP | https://git-scm.com/download/win |
| Python 3 (3.10+) | 渲染可视化图 | https://www.python.org/downloads/ |
| WSL2 (可选) | 替代 VM 的本地 Linux 开发环境 | `wsl --install Ubuntu-22.04` |

安装完 Python 后，在 Git Bash 中装依赖：
```bash
pip install matplotlib numpy pyyaml
```

### VMware 虚拟机 (Ubuntu 22.04)
| 软件 | 位置 |
|------|------|
| ROS2 Humble | `/opt/ros/humble/` |
| VMware 网络 | NAT 模式 |

> **注意**: Fields2Cover 和 OR-Tools 已预编译打包在项目中，VM 端无需单独安装。

### WSL2 (可选，推荐)
| 软件 | 位置 |
|------|------|
| ROS2 Humble | `/opt/ros/humble/` (清华镜像源安装) |
| Fields2Cover | 从源码编译 (`src/Fields2Cover/`) |
| GUI | WSLg 自动支持 (RViz2 可直接运行) |

WSL2 相比 VM 的优势：GPU 加速、启动快、文件互通、无需 SSH 同步。
详细搭建步骤见 [WSL2 环境搭建记录](../.claude/projects/C--WINDOWS-system32/memory/wsl2-f2c-setup.md)。

**WSL2 快速启动：**
```bash
# 在 WSL Ubuntu 终端中
source scripts/wsl_env.sh     # 一键环境配置
bash scripts/batch_test_v2.sh # 运行全量批测
```

---

## 2. 项目结构

```
f2c_coverage_planner/               ← 项目根目录
│
├── README.md                        ← 项目总览 + 基准数据
├── SETUP_GUIDE.md                   ← 你正在读的文档（部署指南）
├── sync_and_build.sh                ← ★ 一键工作流入口（Git Bash 中运行）
│
├── src/yingshi_robot/               ← ROS2 包（需部署到 VM）
│   ├── src/
│   │   ├── planner_core.cpp            主规划器 (核心算法流程)
│   │   ├── polygon_planner_node.cpp    ROS2 节点 + JSON 导出
│   │   ├── decomposer.cpp              Sweep 扫描线分解
│   │   ├── swath_generator.cpp         Swath 生成 + 角度优化 + per-cell veto
│   │   ├── boundary_filler.cpp         边界补刀 + 孔洞检测
│   │   ├── path_planner.cpp            路径规划 (TSP/route/连接修复)
│   │   └── coverage_evaluator.hpp      覆盖评估 + 空隙分类器
│   ├── include/yingshi_robot/         模块头文件 × 8
│   ├── CMakeLists.txt
│   ├── test_polygons/                 S1~S7 测试场景 (YAML)
│   ├── config/f2c_areas/              S8 实际场景
│   └── test/                          GTest 单元测试
│
├── src/Fields2Cover/                 ← Fields2Cover 源码 + 第三方库
│   └── third_party/
│       └── ortools-src/              ★ OR-Tools 预编译库
│           ├── include/               C++ 头文件
│           └── lib/                   库文件 (.so)
│
├── scripts/                           ← 辅助脚本
│   ├── batch_test_v2.sh               批量测试 (8 场景)
│   ├── render_coverage.py             可视化渲染
│   └── s7_quick_test.sh               S7 单场景快速调试
│
├── docs/                              ← 文档
│   ├── F2C_技术文档.md                完整技术文档
│   ├── daily_logs/                    每日开发日志
│   └── references/                    参考资料
│
└── test_results/                      ← 测试结果输出目录
    └── batch_<时间戳>/                 每次批测独立目录
```

---

## 3. VM 端部署（在 VM 上操作）

### 3.1 安装 SSH 服务（如未装）

```bash
sudo apt install -y openssh-server
sudo systemctl enable ssh && sudo systemctl start ssh
```

### 3.2 确认 IP

```bash
ip addr show | grep "inet " | grep -v 127.0.0.1
# 输出示例: inet 192.168.83.129/24 ...
```

> 📝 记下这个 IP。

### 3.3 部署源码到 VM

将 Windows 上的整个 `f2c_coverage_planner` 文件夹传到 VM 的 home 目录。

```bash
# 备份旧版（如果有）
mv ~/f2c_coverage_planner ~/f2c_coverage_planner_backup 2>/dev/null || true

# 将文件夹复制到 home 目录
cp -r ~/桌面/f2c_coverage_planner ~/

# 验证关键文件
ls ~/f2c_coverage_planner/src/yingshi_robot/CMakeLists.txt
# 应看到: CMakeLists.txt
ls ~/f2c_coverage_planner/install/fields2cover/lib/libFields2Cover.so
# 应看到: libFields2Cover.so
```

### 3.4 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/f2c_coverage_planner
colcon build --packages-select yingshi_robot --symlink-install
```

看到 `Summary: 1 package finished` = 成功 ✅

### 3.5 永久环境配置

```bash
cat >> ~/.bashrc << 'EOF'

# === F2C Coverage Planner ===
source /opt/ros/humble/setup.bash
source ~/f2c_coverage_planner/install/setup.bash
export LD_LIBRARY_PATH=$HOME/f2c_coverage_planner/install/fields2cover/lib:$HOME/f2c_coverage_planner/install/lib:$HOME/f2c_coverage_planner/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH}
EOF

source ~/.bashrc
```

> ⚠️ `LD_LIBRARY_PATH` 必须包含三个路径：`install/fields2cover/lib`、`install/lib`、`ortools-src/lib`

### 3.6 修复脚本换行符（Windows 同步后必做）

从 Windows Git Bash 同步脚本到 VM 后，行尾可能是 CRLF，导致 bash 报错：

```bash
sed -i 's/\r//' ~/f2c_coverage_planner/scripts/*.sh
```

---

## 4. Windows 端配置 SSH（在本机上操作）

### 4.1 生成密钥 + 免密登录

右键桌面 → **Git Bash Here**：

```bash
ssh-keygen -t rsa -b 4096 -f ~/.ssh/id_rsa -N ""
ssh-copy-id dc@192.168.83.129    # 需要输一次 VM 密码
```

### 4.2 验证

```bash
ssh dc@192.168.83.129 "echo '连接成功!'"
```

### 4.3 修改 VM 连接信息

用记事本打开 `sync_and_build.sh`，修改第 17 行：

```bash
VM_HOST="dc@你的IP地址"
```

---

## 5. 日常使用

> 以下命令在 **Windows Git Bash** 中，`cd` 到 `f2c_coverage_planner/` 目录下执行。

### 5.1 修改代码后快速验证（单场景）

```bash
bash sync_and_build.sh --test S7
```

自动：同步源码 → VM 编译 → 跑 S7 → 输出覆盖率/得分。

### 5.2 8 场景全量基准（生成报告 + 可视化）

```bash
bash sync_and_build.sh --batch
```

自动：同步 → 编译 → 8 场景逐个测试 → 渲染路径图+热力图 → 拉回 Windows。

结果在 `test_results/batch_<时间戳>/` 中：
| 文件 | 说明 |
|------|------|
| `*_coverage.png` | 路径规划图 |
| `*_cells.png` | Cell 分解图 |
| `*_connections.png` | 连接关系图 |
| `*_data.json` | 路径点 + 评估数据 |
| `batch_report.txt` | 汇总报告 |

### 5.3 在 VM 上看 RViz 可视化

在 **VMware 虚拟机窗口** 中操作：

```bash
# 修复 VMware 黑块问题（一次性）
export LIBGL_ALWAYS_SOFTWARE=1

# 启动 S8 场景 RViz
bash ~/f2c_coverage_planner/install/yingshi_robot/share/yingshi_robot/scripts/run_f2c_optimized.sh
```

你会看到：黄色多边形 + 灰色孔洞 + 橙色规划路径。

### 5.4 仅同步 + 编译（不测试）

```bash
bash sync_and_build.sh
```

---

## 6. 测试场景说明

| 场景 | 描述 | 面积 | 孔洞数 | 挑战点 |
|------|------|:--:|:-----:|--------|
| S1 | 矩形 20×15m | 300 m² | 0 | 边界覆盖完整性 |
| S2 | L 形 | 397 m² | 0 | 非凸形状分解 |
| S3 | 含孔洞 | 458 m² | 3 | 孔洞绕行 |
| S4 | 窄走廊 | 40 m² | 0 | 窄通道转弯受限 |
| S5 | 不规则 | 90 m² | 0 | 复杂轮廓 |
| S6 | 多区域 | 84 m² | 2 | 多区域连接 |
| S7 | 工厂车间 30×20m | 501 m² | 10 | 多孔洞复杂障碍物 |
| S8 | L 形缺口+中心孔 | 297 m² | 1 | 缺口边界+孔洞 |

---

## 7. 常见问题

### ❌ 编译报错 `Could not find Fields2Cover`

```bash
ls ~/f2c_coverage_planner/install/fields2cover/lib/cmake/Fields2Cover/Fields2CoverConfig.cmake
```
如果文件不存在，在 VM 上重新编译 Fields2Cover：
```bash
cd ~/f2c_coverage_planner/src/Fields2Cover
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
cmake --install . --prefix ~/f2c_coverage_planner/install/fields2cover
```

### ❌ `error while loading shared libraries`

```bash
source ~/.bashrc
# 或手动：
export LD_LIBRARY_PATH=$HOME/f2c_coverage_planner/install/fields2cover/lib:$HOME/f2c_coverage_planner/install/lib:$HOME/f2c_coverage_planner/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH}
```

### ❌ 批量测试卡住 / 进程残留

```bash
pkill -f polygon_planner_node
```

### ❌ 脚本运行报 `$'\r': 未找到命令`

Windows 同步后行尾是 CRLF，修复：
```bash
sed -i 's/\r//' ~/f2c_coverage_planner/scripts/*.sh
```

### ❌ RViz 界面黑块

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

### ❌ `ssh: connect to host port 22: Connection refused`

VM SSH 没启或 IP 变了：
```bash
sudo systemctl restart ssh
ip addr show | grep "inet 192"
```

---

> 🤖 F2C Coverage Planner v9.11 — 2026-07-20
> GitHub: [wwwyw-dc-51/f2c_coverage_planner](https://github.com/wwwyw-dc-51/f2c_coverage_planner)
> 一键批测: `bash sync_and_build.sh --batch` | 技术文档: `docs/F2C_技术文档.md`
