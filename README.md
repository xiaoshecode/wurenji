# wurenji

“**We are 昌平队**”的 ROS 1 无人机视觉穿环实验代码，包含图像发布、飞控服务、环检测与 RTSP 解码示例。默认开发分支为 `dev_passing_ring`。

[在线文档](https://xiaoshecode.github.io/xiaoshecode/repos/wurenji/) · [相机节点说明](raspicam_node/README.md) · [图像发布说明](image_pub/README.md)

## 目录

| 路径 | 作用 |
| --- | --- |
| `raspicam_node/` | ROS 包 `ttauav_node`，消息/服务、飞控通信和视觉脚本 |
| `raspicam_node/src/cv/circle_detect.py` | 离线紫色圆环检测逻辑 |
| `raspicam_node/src/test_scripts/pass_ring.py` | ROS 穿环状态机，会发送真实飞行命令 |
| `image_pub/` | ROS 图像发布包，依赖厂商 RTSP 共享库 |
| `Decodertest/` | 独立解码器示例、头文件和厂商 `.so` |
| 根目录 `*.sh` | 指定机器环境中的启动脚本 |
| `tests/` | 不连接 ROS、不发飞行指令的视觉回归测试 |

保留协议生成头文件 `ttalink/`、第三方库和 ROS 包边界，避免按扩展名移动源文件破坏 include、launch 与 CMake 引用。`raspicam_node/LICENSE` 中保留相机节点上游许可证；不能将其自动视为厂商二进制的授权。

## 构建前提

目标环境为 Linux + ROS 1/catkin、C++14、OpenCV、匹配 CPU 架构的厂商 SDK/`.so`，原工程使用 ROS Melodic 路径。当前代码还涉及 NumPy、cv_bridge、pyzbar、apriltag 等按具体脚本选用的 Python 依赖；不同 ROS 发行版的 Python 版本需分别匹配。

仓库根目录 CMake 原先是指向 Windows 绝对路径的失效符号链接，现已替换成 **Git 普通文件（模式 100644）**，通过已加载 ROS 环境查找 catkin。以下展示将本仓库作为 `catkin_ws/src` 的布局：

```sh
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws
cd ~/catkin_ws
git clone --branch dev_passing_ring https://github.com/xiaoshecode/wurenji.git src
catkin_make
source devel/setup.bash
```

编译前先满足 `image_pub/package.xml`、`raspicam_node/package.xml` 依赖，确认厂商库适配目标板。Windows 不能直接运行仓库中的 Linux `.so`。本次未验证完整 catkin 构建。

## 现有启动脚本

每个长期运行的服务使用独立终端，先检查脚本中的 `/root/catkin_ws`、`/root/tta_ros` 和 SDK 程序路径：

1. `bash startpsdk.sh`：厂商 SDK 服务。
2. `bash startroscore.sh`：ROS master。
3. `bash startuavservice.sh`：飞控服务。
4. `bash startimage.sh`：图像发布。
5. `bash pass_ring.sh`：穿环控制，**会使无人机起飞**。

旧 README 中的 `startimagepub.sh`、`startpass_ring.sh` 不存在，已纠正。按原项目要求，真实飞行必须在合适场地由操作者持遥控器随时接管。本次检查没有执行任何服务或飞行脚本。

## 离线测试

以下只测试图像算法，建议使用独立 Python 3 虚拟环境，不改动 ROS 的 Python 环境：

```sh
python -m pip install numpy opencv-python-headless
python -m unittest discover -s tests -v
```

2026-10-30：4 项回归测试通过，覆盖全前景图像保留、空图像、多个候选匹配时删除正确的双向关系，以及多目标图像的既有选择顺序。源码也完成 Python AST 解析；这不等于 ROS 导入/运行验证。

本次修复：以连通域标签 0 识别背景，并保留原来的前景包围盒面积稳定排序（调用方使用 `circles[0]`）；修正多匹配关系删除索引；穿环脚本使用相对源码路径导入；修正 `demo2.py` 的空函数语法错误（仍显式标记未实现）；修复根 CMake 入口及文件类型，移除没有配置文件的 dynamic_reconfigure 空调用和不存在的 include 导出。

## 已知限制

飞控服务、ROS 消息生成、RTSP、厂商库 ABI、整机行为均未验证。若干旧脚本仍有机器绝对路径及 Python 2 风格 shebang，需在部署机逐个配置；`demo2.py` 是未完成示例，不是可运行穿环程序。算法测试不验证飞行控制稳定性。新增文本遵循 `.editorconfig`，生成缓存和构建目录由 `.gitignore` 排除。
