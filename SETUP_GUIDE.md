# F2C 全覆盖路径规划 — 配置与使用指南

> **版本**: v9 模块化重构版 (2026-07-14)
> **环境**: Windows 11 宿主机 + Ubuntu 22.04 VMware 虚拟机 (ROS2 Humble)

---

## 目录

- [1. 环境要求](#1-环境要求)
- [2. 解压与文件结构](#2-解压与文件结构)
- [3. VM 端部署](#3-vm-端部署在-vm-上操作)
- [4. Windows 端配置 SSH](#4-windows-端配置-ssh在本机上操作)
- [5. 日常使用](#5-日常使用)
- [6. 常见问题](#6-常见问题)

---

## 1. 环境要求

### Windows 宿主机
| 软件 | 用途 | 下载 |
|------|------|------|
| Git Bash | 运行脚本、SSH/SCP | https://git-scm.com/download/win |
| Python 3 | 渲染可视化图 | https://www.python.org/downloads/ |

安装完 Python 后，在 Git Bash 中装依赖：
```bash
pip install matplotlib numpy pyyaml
```

### VMware 虚拟机 (Ubuntu 22.04)
| 软件 | 位置 |
|------|------|
| ROS2 Humble | `/opt/ros/humble/` |
| VMware 网络 | NAT 模式 |

> **注意**: Fields2Cover 和 OR-Tools 已预编译打包在发布包中，VM 端无需单独安装。

---

## 2. 解压与文件结构

将 `f2c_coverage_planner.zip` 解压到 Windows 桌面，得到：

```
f2c_coverage_planner/               ← 项目根目录
│
├── SETUP_GUIDE.md                  ← 你正在读的文档
├── sync_and_build.sh               ← ★ 一键工作流入口（Git Bash 中运行）
│
├── install/fields2cover/           ← ★ Fields2Cover 预编译库（已编译好，直接可用）
│   ├── include/fields2cover/       C++ 头文件
│   ├── lib/
│   │   ├── libFields2Cover.so      核心库
│   │   ├── libsteering_functions.so
│   │   ├── libmatplot.so           可视化库
│   │   └── cmake/Fields2Cover/     CMake 配置文件
│   │       ├── Fields2CoverConfig.cmake
│   │       ├── Fields2CoverTargets.cmake
│   │       └── Fields2CoverTargets-release.cmake
│
├── src/yingshi_robot/              ← ROS2 包（需复制到 VM）
│   ├── src/
│   │   ├── polygon_planner_node.cpp   主节点 (3930行)
│   │   ├── coverage_evaluator.hpp     评估器
│   │   ├── decomposer.cpp             区域分解模块
│   │   ├── swath_generator.cpp        Swath生成模块
│   │   ├── boundary_filler.cpp        边界补刀模块
│   │   └── path_planner.cpp           路径规划模块
│   ├── include/yingshi_robot/        模块头文件 × 5
│   ├── CMakeLists.txt
│   ├── test_polygons/                S1~S6 测试场景 (YAML)
│   └── config/f2c_areas/             notched 等实际场景
│
├── src/Fields2Cover/               ← Fields2Cover 源码 + 第三方库
│   └── third_party/
│       └── ortools-src/            ★ OR-Tools 预编译库
│           ├── include/             C++ 头文件
│           └── lib/                 库文件 (.so)
│
├── scripts/                         ← 辅助脚本
│   ├── batch_test_v2.sh             批量测试
│   ├── render_coverage.py           可视化渲染
│   └── sync_and_build.sh           一键工作流（根目录也有副本）
│
└── test_results/                    ← 预生成的示例结果
    └── *.png                        S1~S6 + notched 覆盖图
```

> **关键**: 发布包已包含预编译的 Fields2Cover 和 OR-Tools，用户无需在 VM 上编译 Fields2Cover，只需编译 yingshi_robot 即可。

---

## 3. VM 端部署（在 VM 上操作）

> 以下操作在 **VMware 虚拟机窗口** 或 **SSH 终端** 中完成。

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

通过 VMware "拖放" 或 "共享文件夹"，将 Windows 上解压的 **整个 `f2c_coverage_planner` 文件夹** 传到 VM 桌面。

在 VM 终端：

```bash
# 备份旧版（如果有）
mv ~/f2c_coverage_planner ~/f2c_coverage_planner_backup 2>/dev/null || true

# 将整个文件夹复制到 home 目录
cp -r ~/桌面/f2c_coverage_planner ~/

# 验证关键文件
ls ~/f2c_coverage_planner/install/fields2cover/lib/libFields2Cover.so
# 应看到: libFields2Cover.so
ls ~/f2c_coverage_planner/install/fields2cover/lib/cmake/Fields2Cover/Fields2CoverTargets.cmake
# 应看到: Fields2CoverTargets.cmake
ls ~/f2c_coverage_planner/src/Fields2Cover/third_party/ortools-src/include/
# 应看到: absl/ dijkstra/ eigen3/ 等目录
```

### 3.4 编译（一步到位）

```bash
source /opt/ros/humble/setup.bash
cd ~/f2c_coverage_planner
colcon build --packages-select yingshi_robot --symlink-install
```

看到 `Summary: 1 package finished` = 成功 ✅

> **为什么不需要编译 Fields2Cover？** 发布包中的 `install/fields2cover/` 已包含预编译的 .so 库和 CMake 配置文件，CMakeLists.txt 已配置好路径，colcon 会自动找到它。

### 3.5 永久配置（以后开终端自动生效）

```bash
cat >> ~/.bashrc << 'EOF'

# === F2C Coverage Planner 环境 ===
source /opt/ros/humble/setup.bash
source ~/f2c_coverage_planner/install/setup.bash
export LD_LIBRARY_PATH=$HOME/f2c_coverage_planner/install/fields2cover/lib:$HOME/f2c_coverage_planner/install/lib:$HOME/f2c_coverage_planner/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH}
EOF

source ~/.bashrc
```

> ⚠️ `LD_LIBRARY_PATH` 必须包含三个路径：
> 1. `install/fields2cover/lib` — Fields2Cover 自身 .so
> 2. `install/lib` — yingshi_robot 的 .so
> 3. `src/Fields2Cover/third_party/ortools-src/lib` — OR-Tools .so

---

## 4. Windows 端配置 SSH（在本机上操作）

### 4.1 生成密钥 + 免密登录

右键桌面 → **Git Bash Here**，输入：

```bash
# 生成密钥（一路回车）
ssh-keygen -t rsa -b 4096 -f ~/.ssh/id_rsa -N ""

# 复制到 VM（需要输一次 VM 密码）
ssh-copy-id dc@192.168.83.129
```

> ⚠️ `192.168.83.129` 换成你 VM 的实际 IP。用户名不是 `dc` 的话也换掉。

### 4.2 验证

```bash
ssh dc@192.168.83.129 "echo '连接成功!'"
```

看到 "连接成功!" 即可。

### 4.3 如果 IP 不同，改一行配置

用记事本打开 `sync_and_build.sh`，修改第 16 行：

```bash
VM_HOST="dc@你的IP地址"
```

---

## 5. 日常使用

> 以下命令在 **Windows Git Bash** 中，`cd` 到 `f2c_coverage_planner/` 目录下执行。

### 5.1 修改代码后快速验证

```bash
bash sync_and_build.sh --test S1
```

自动：同步 → 编译 → 跑 S1 → 输出覆盖率/得分。

### 5.2 7 场景全量基准（生成报告 + 可视化）

```bash
bash sync_and_build.sh --batch
```

自动：同步 → 编译 → 7 场景逐个测试 → 每场景渲染路径图+热力图 → 拉回 Windows。

结果在 `test_results/batch_<时间戳>/` 中：
- `*_coverage.png` — 可视化图
- `*_data.json` — 原始数据

### 5.3 在 VM 上看 RViz 界面

**方式一**（推荐）：在 **VMware 虚拟机窗口** 直接操作

```bash
# 先修复 VMware 黑块问题（一次性）
export LIBGL_ALWAYS_SOFTWARE=1

# 启动 notched 场景 RViz
bash ~/f2c_coverage_planner/install/yingshi_robot/share/yingshi_robot/scripts/run_f2c_optimized.sh
```

你会看到 3D 视图：黄色多边形 + 灰色孔洞 + 橙色规划路径。按 `Ctrl+C` 停止。

**方式二**：从 Windows 远程触发（需 VM 窗口打开才看得到 RViz）

```bash
bash sync_and_build.sh --run
```

### 5.4 仅同步 + 编译（不测试）

```bash
bash sync_and_build.sh
```

### 5.5 理解可视化图

打开任意 `*_coverage.png`：

| 左侧：路径规划图 | 右侧：覆盖热力图 |
|------|------|
| 黄色底 = 多边形 | 黄色底 = 多边形 |
| 灰色块 = 孔洞 | 灰色块 = 孔洞 |
| 橙色线 = 规划路径 | 绿色点 = 已覆盖 |
| | 红色点 = 未覆盖 |
| | **右上角标签 = 覆盖率 + 得分** |

---

## 6. 常见问题

### ❌ 编译报错 `Could not find a package configuration file provided by "Fields2Cover"`

原因：`install/fields2cover/` 目录不存在或路径不对。

检查：
```bash
ls ~/f2c_coverage_planner/install/fields2cover/lib/cmake/Fields2Cover/Fields2CoverConfig.cmake
```
如果文件不存在，说明发布包中的预编译库未正确复制到 VM。重新执行第 3.3 节。

### ❌ `error while loading shared libraries: libFields2Cover.so`

LD_LIBRARY_PATH 没设或缺少路径。
```bash
source ~/.bashrc
# 或手动：
export LD_LIBRARY_PATH=$HOME/f2c_coverage_planner/install/fields2cover/lib:$HOME/f2c_coverage_planner/install/lib:$HOME/f2c_coverage_planner/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH}
```

### ❌ `error while loading shared libraries: libortools.so.9`

OR-Tools 库路径不在 LD_LIBRARY_PATH 中。确保包含：
```bash
$HOME/f2c_coverage_planner/src/Fields2Cover/third_party/ortools-src/lib
```

### ❌ `ssh: connect to host ... port 22: Connection refused`

VM SSH 没启或 IP 变了。

```bash
# VM 终端
sudo systemctl restart ssh
ip addr show | grep "inet 192"
```

IP 变了 → 改 `sync_and_build.sh` 第 16 行。

### ❌ RViz 界面黑块

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

### ❌ `colcon: command not found`

```bash
source /opt/ros/humble/setup.bash
```

### ❌ 批量测试中途卡住

VM 终端清理僵尸进程：
```bash
pkill -f polygon_planner
```

### ❌ 用户名不是 `dc`

在 `sync_and_build.sh` 和本指南中，把所有 `dc@` 替换为 `你的用户名@`。

### ❌ CMake 报错 `CMakeCache.txt directory is different`

VM 上残留了旧路径的编译缓存。清除后重编：

```bash
rm -rf ~/f2c_coverage_planner/build ~/f2c_coverage_planner/install/yingshi_robot
cd ~/f2c_coverage_planner
colcon build --packages-select yingshi_robot --symlink-install
```

> ⚠️ 注意：只删 `install/yingshi_robot`，不要删 `install/fields2cover`！

### ❌ 编译报错 `include could not find Fields2CoverTargets.cmake`

这是旧版发布包的已知问题（缺少安装产物）。新版发布包已修复。如果遇到：
1. 确认发布包中有 `install/fields2cover/lib/cmake/Fields2Cover/Fields2CoverTargets.cmake`
2. 如果没有，需要在 VM 上重新编译 Fields2Cover（见下方补充步骤）

<details>
<summary>补充：在 VM 上手动编译 Fields2Cover（仅旧版发布包需要）</summary>

```bash
cd ~/f2c_coverage_planner/src/Fields2Cover
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
cmake --install . --prefix ~/f2c_coverage_planner/install/fields2cover

# 然后编译 yingshi_robot
cd ~/f2c_coverage_planner
rm -rf build
colcon build --packages-select yingshi_robot --symlink-install
```
</details>

---

> 🤖 F2C Coverage Planner v9 — 2026-07-14
> 一键: `bash sync_and_build.sh --batch` | 技术文档: `docs/F2C_技术文档.docx`
