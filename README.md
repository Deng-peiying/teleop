# Wuji Glove 遥操作

## Wuji Hand 2 当前滑行方法

当前仅提供 `thumb_track_b4`。官方 Wuji SDK 每帧生成 20 关节参考，拇指四关节通过 SLSQP 做局部接触修正，其余四指沿用 SDK 输出。接触区域采用连续网格局部坐标，保留指腹转向与往返滑行。

方法根据骨架几何估计接触权重。估计存在正间隙时权重为零，进入估计皮肤区域后逐渐增加。目标包括表面间隙、切向运动、接触法向及相对 SDK 修正量的连续性。回滑途中退出接触时，第一轴的历史修正撤回受到方向限制，并通过网格间隙检查后才输出，以避免向反方向回弹。

### 安装与运行

使用 Python 3.12，在 WSL Ubuntu 环境从仓库根目录运行：

```bash
python3.12 -m venv .venv
.venv/bin/pip install -r requirements-hand2.txt
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m teleop.hand2_live --method thumb_track_b4
```

Windows PowerShell 可运行 `./start.ps1`，它使用 Ubuntu-22.04 和本仓库的 `.venv`。手套默认地址为 `192.168.1.100:50001`，SDK 用户为 `operator_left`。用户需在 SDK 中建立并标定该用户。命令行通过 `--address` 和 `--user` 覆盖配置。

窗口显示固定手腕的关节指令，记录保存到 `runs/hand2_live/`。当前版本经过左手手套试戴，用户认可食指侧缘往返滑行效果。接触权重仍依赖估计皮肤厚度，真实接触位置与其他三指的完整动作仍需试戴评估。

核心代码位于 `src/teleop/thumb_relative.py`、`surface_tracker.py`、`thumb_patch.py` 和 `thumb_gradient.py`。模型资源和第三方许可证位于 `models/` 与 `drivers/`。本次源码整理使用独立初始提交。

## 其他已有设备功能

以下保留仓库已有的 L20 和机械臂使用说明。

# Wuji 手套 → LinkerHand L20 右手

SDK、Qt、模型和自己的集成代码都放在本项目内。运行不引用旁边的 `linkerhand-sdk`、`wuji-sdk` 或学长仓库。移动项目后重新创建 `.venv`，不要搬运虚拟环境。

## 目录

```text
teleop/
├── drivers/
│   ├── linkerbot-hand-python-sdk/  新版官方 Python SDK，L20 的 16 通道读写
│   ├── linkerbot-transport/        新版 SDK 的 CAN/串口传输依赖
│   ├── realman-rm-api2/            睿尔曼官方仓库，Python SDK 位于 Python/Robotic_Arm
│   ├── wuji-sdk/                   官方文档与示例仓库
│   └── wheels/                    官方 Wuji Python wheel；mjlab wheel
├── src/teleop/
│   ├── hand.py                    新版 SDK 的连接、命名通道读写
│   ├── glove.py                   Wuji 扫描、连接、订阅最新帧
│   ├── mapping.py                 共用的 L20 通道、URDF 路径和 raw 转换
│   ├── retarget.py                Wuji 21→L20 16 全手重定向
│   ├── cli.py                     hand / glove / follow / follow-all 命令
│   ├── qt.py                      复用官方布局、调用新版 SDK 的 L20 调试界面
│   ├── arm.py                     RM75 SDK 连接与最新目标跟随
│   ├── mujoco_model.py            arm/hand 联合模型与网格命名空间
│   └── mujoco_control.py          arm、hand、arm-hand 三个调试入口
├── models/L20/right/              官方右手 URDF 和 meshes
├── models/Wuji/                   官方 SDK 内置右手运动学 URDF 和许可证
├── models/RM75/                   RM75-6FB 原始 URDF、网格与安装参数
├── profiles/table_facing.json     当前实际设备读数保存的桌面参考姿态
├── tests/                         联合模型一致性回归检查
├── scripts/
│   ├── view_l20_mjlab.py           纯仿真关节控制
│   ├── sim_to_hand.py              仿真滑块 → 新版官方 SDK → 真机
│   └── start_qt.ps1 / start_qt.sh  桌面 Qt 启动入口
├── requirements.txt               手套和新版 L20 SDK
├── requirements-qt.txt            加装 Qt 依赖
└── requirements-sim.txt           加装 L20 仿真依赖
```

官方代码、文档和许可证保留在 `drivers`，业务代码在 `src/teleop`。Wuji 仓库不包含完整 Python 实现源码，所以另存官方 `2026.10.26` CPython 3.12 Linux x86_64 wheel；安装时使用本地文件。新版 LinkerHand `2.0.0` 及 transport 从项目内源码安装，mjlab `1.6.0` 从本地 wheel 安装。没有复制学长的 ROS2 驱动，因为当前调用链使用官方 Python SDK。

这里的 wheel（`.whl`）是官方预编译安装包，不是我们编写的替代驱动。该包包含 `wuji_sdk/__init__.py`、接口声明 `__init__.pyi` 和已编译的 `wuji_sdk.cpython-312-x86_64-linux-gnu.so`。GitHub 仓库主要提供示例与文档，安装包才提供运行时实现；“没有完整源码”不等于“SDK 功能不完整”。文件名的 `2026.10.26` 是版本号，`cp312` 对应 CPython 3.12，Linux x86_64 对应本机 WSL 的系统与处理器架构。学习接口时看官方示例和 `.pyi`；底层编译实现不能当作普通 `.py` 源码阅读。

## 与论文复现项目的边界

`teleop` 负责 Wuji/L20 SDK 集成、遥操、手动调试和模型检查。HORA 等论文项目在同级目录独立维护，不放入 `teleop/third_party`，也不由 teleop 启动训练。

迁移 L20 时，将模型目录（URDF、meshes、许可证）放入论文项目的 `assets`，然后在论文项目内适配主动关节顺序、mimic、actuator、动作/观测维度、限位和任务逻辑；仅替换模型文件通常不足以完成复现。IsaacLab/MuJoCo 所需的模型转换和训练配置也归论文项目管理，不能让训练代码反向依赖 teleop。

本次已把原有 HORA 实验入口移至独立 `hora/scripts/l20_mjlab.py`，模型构建器和资产分别放在该项目的 `scripts/l20_model.py` 与 `assets/l20`。这是既有实验代码的归位，不代表已完成论文的 L20 复现。本次不运行训练。

## 安装：WSL Ubuntu-22.04，Python 3.12

```bash
cd '/mnt/d/data/learning/EMG+灵巧手/teleop'
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r requirements.txt
source .venv/bin/activate
python -m teleop --help
```

需要 Qt 或仿真时，分别安装：

```bash
uv pip install --python .venv/bin/python -r requirements-qt.txt
uv pip install --python .venv/bin/python -r requirements-sim.txt --torch-backend cu132
```

普通第三方依赖仍需联网安装。仿真包括 PyTorch 等较大依赖。本地 Wuji wheel 适用于上述 WSL 环境，不适用于 Windows 原生 Python。

## 读设备，再连接两端

USB-CAN 需通过 usbipd 接入 WSL。`ip -details link show can0` 应能看到接口。设备使用 1 Mbps，需要配置时，在没有其他控制程序运行的情况下执行：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 listen-only off
sudo ip link set can0 up
python -m teleop hand
```

`hand` 只读取身份和 16 个命名通道，要求 L20 right。此前识别的序列号为 `LHT20-011-060-R-B-1-D`。不要同时运行 Qt、仿真联动和遥操写入程序。

手套接入 WSL 后：

```bash
python -m teleop glove
# 多只手套时指定序列号
python -m teleop glove --sn 实际序列号
```

输出 finger 编号、angles 分量和 confidence。实际伸屈食指确认 finger/component，并记录伸直与弯曲的 angle，不能凭编号猜关节。

单通道映射预览格式如下，变量须替换为实际确认的值：

```bash
python -m teleop follow --finger "$FINGER" --component "$COMPONENT" \
  --channel index_tip --open-angle "$OPEN_ANGLE" --bent-angle "$BENT_ANGLE" \
  --raw-open "$RAW_OPEN" --raw-bent "$RAW_BENT"
```

默认只显示映射目标与真机回读。现场确认后，同一命令加 `--live` 才发送。当前只选一个食指通道（`index_root/index_tip/index_abd`），其他通道目标取自启动回读。Ctrl+C 结束发送，不代表电机断电。手套数据与实物联动仍需接入手套后验收。

原 `follow` 保留作单通道端点验证；全手使用 `follow-all`，映射规则见下节。两者都调用新版 SDK，不需要 Qt。

## Wuji 21→L20 16 全手映射

```bash
# 手套与 L20 均连接时，先预览全部 16 通道，不写入
python -m teleop follow-all
# 现场运行并写入真机
python -m teleop follow-all --live
# 多只手套时可加 --sn 实际序列号
```

默认 20 Hz，沿用现有的 `--stale-seconds 0.5` 和 `--min-confidence 0.5`。没有新帧、某指置信度不足或输入不合法时结束跟随，不继续发送旧目标。关闭程序不会给电机断电。首次软件验证没有连接手套或发送真机动作。

官方接口定义的是手指顺序、关节含义和模型约束，**并没有厂家提供的 Wuji→L20 跨品牌映射表**。`retarget.py` 根据这些定义实现了下面的运动学重定向；这不等于完成了人体手型或实物角度标定。

Wuji 帧是 5×5 槽，按拇指、食指、中指、无名指、小指排列，单位为 rad。有效分量共 21 个，其余四指最后一槽必须为 0：

| 手指 | 槽 0 | 槽 1 | 槽 2 | 槽 3 | 槽 4 |
|---|---|---|---|---|---|
| 拇指 | CMC rot，Z 轴 | CMC flex，X 轴 | CMC abd，Y 轴 | MCP 屈伸，X 轴 | IP 屈伸，X 轴 |
| 食/中/无名/小指 | MCP flex，X 轴 | MCP abd，Y 轴 | PIP，X 轴 | DIP，X 轴 | 0 占位 |

槽位名称、轴和运动链核对了 [Wuji 官方角度接口](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/hand-tracking)、[官方右手 URDF](https://github.com/wuji-technology/wuji-description/blob/main/glove/body/urdf/right.urdf) 与已安装的 `wuji-sdk 2026.10.26` 内置右手模型。公开仓库当前模型与 SDK 内置模型的手指长度、线圈位置存在差异，所以 `models/Wuji/right.urdf` 固定保存了此版本 SDK 动态库内嵌的完整 `right_hand` URDF，而不是混用另一版几何。它用于读取运动学定义；本项目不加载其可视化网格。许可证来自官方 `wuji-description` 仓库。标定模型只要沿用这些关节轴序与定义即可使用本映射；自行改变轴序的模型不能直接套用。

### 拇指：保留 3 个 CMC 输入，合并 MCP/IP

L20 有 3 个主动 CMC 关节，按模型链顺序连接 SDK 的 `thumb_roll / thumb_rotation / thumb_root`。因此不能简单丢弃 Wuji 某个 CMC 槽，也不能仅按英文名字把三个角直接复制过去：L20 的关节轴和固定旋转不同。

代码按两套 URDF 的固定旋转与关节轴构建 CMC 旋转矩阵，在 L20 三轴限位内寻找最接近的人手 CMC 朝向。局部解剖坐标关联为：Wuji 屈伸轴 X 对应 L20 屈伸轴 Y，Wuji 指骨远端 −Z 对应 L20 +Z；两套模型的零姿态用于建立其余固定对齐。源 CMC 的三个有效角都参与目标朝向计算。若朝向超出 L20 可表示范围，取限位内的近似；奇异姿态附近用前一解与很小的连续性项选择解。该局部坐标关联是本项目的名义模型对齐，不是厂家实测标定。

L20 的 `thumb_dip` 跟随 `thumb_mcp`，按手册端点线性插值，当前为 `k = 71/70`。Wuji 的 MCP/IP 两个角则共同投影成一个 `thumb_tip` 主动目标，不会丢弃 IP。

### 四指：MCP 两轴保留，PIP/DIP 投影到联动约束

| L20 通道 | Wuji 输入 |
|---|---|
| index/middle/ring/pinky_root | 对应手指槽 0：MCP flex |
| index/middle/ring/pinky_abd | 对应手指槽 1：MCP abd |
| index/middle/ring/pinky_tip | 对应手指槽 2、3：PIP 与 DIP 的约束投影 |

四指当前模型采用 `DIP = k × PIP + offset`，当前 `k = 78/99`、`offset = 0`。对于人手的两个角 `a=PIP`、`b=DIP`，选择主动角 `q`，使两关节角度误差平方和最小：

```text
minimize (q − a)² + (kq + offset − b)²
q = [a + k(b − offset)] / (1 + k²)
```

同一公式也用于拇指 MCP/IP。之后按主关节和被动关节两者的手册限位共同裁剪。若输入已经满足 mimic，输出保留原主动角；否则取折中，而不是只用 PIP、直接相加或把 DIP 当第二个独立电机。限位来自手册，offset 取 0。

最后将 16 个主动弧度按 `mapping.py` 的共用规则转成新版 SDK raw，调用 `hand.write_named()` 下发。SDK 通道顺序按 capabilities 读取，不使用旧 Qt 的 20 槽索引。

### 没有设备时验证

用模拟手套帧打开 MuJoCo，直接观察全手重定向（不会连接设备或发送 CAN）：

```bash
python scripts/preview_glove_mapping.py
```

按“零姿态 → 拇指三轴 CMC → 拇指 MCP/IP → 食指、中指、无名指、小指各自根部与末端 → 全手”循环，每轮约 44 秒。模拟数据遵守 Wuji 的 5×5 槽位格式，经实际 `WujiToL20.map()` 生成 16 通道 raw，再转回模型弧度送入 mjlab 物理仿真；被动关节由模型 mimic 约束运动。输入是人工构造的角度帧，不是设备采样。MuJoCo 滑块显示脚本目标，关闭窗口结束预览。

## MuJoCo 手动检查：滑块、弧度与 raw

纯仿真入口为 `python scripts/view_l20_mjlab.py`，不连接手套或 L20。不要用 `sim_to_hand.py` 做本节检查，因为后者会把滑块目标发给真机。展开 MuJoCo 的 **Control**；这里滑块是 actuator 的目标角，单位为 rad，不是 0–255，也不一定等于物理仿真瞬间的实际关节角。

真机换算默认使用《Linker Hand L20 产品手册》V1.4（2026-09-14）§1.3，正文第 4–5 页（PDF 第 8–9 页）。V1.3 修订了关节角度。以下为右手，四指同名轴共用范围。

| Qt / MuJoCo / 项目 URDF 名 | 新版 SDK 通道 | 官方原文件关节（仅溯源） | 手册行程（°） |
|---|---|---|---|
| thumb_cmc_roll | thumb_roll | thumb_cmc_yaw | 77.6 |
| thumb_cmc_yaw | thumb_rotation | thumb_cmc_roll | 91 |
| thumb_cmc_pitch | thumb_root | thumb_cmc_pitch | 45 |
| thumb_mcp | thumb_tip | thumb_mcp | 70 |
| *_mcp_roll | *_abd | *_mcp_roll | −13 至 +13 |
| *_mcp_pitch | *_root | *_mcp_pitch | 70 |
| *_pip | *_tip | *_pip | 99 |

运行时加载同目录的 `l20_right_manual.urdf`。其关节名与 Qt、MuJoCo 显示名一致，限位与联动参数已写入文件。

四指的 `*` 依次为 index、middle、ring、pinky。Qt 中文标签括号内的名称与 MuJoCo 相同。SDK 使用同一个位置接口 `hand.set_angles(...)`，由 `hand.write_named()` 按设备的 `joint_names` 排列目标。表中第二列是通道标识。

真机换算采用上表行程，单向行程从项目名义零位起算，侧摆零位取中点。当前项目方向为角度下限 → raw 255，上限 → raw 0，中间线性插值并裁剪。手册给的是产品规格，没有逐台 raw/角度标定曲线。Qt 保持 SDK raw 控制。模拟关节、滑块、手套和真机默认换算共用手册范围；每个主动滑块两端分别覆盖 raw 255 和 0。

被动关节 IP（模型 thumb_dip）在第 4 页图标 70°、第 5 页表列 71°；本文记录表值 71° 并保留图表差异。四指 DIP 表列 78°。二者无独立 SDK 通道。手册没有提供完整联动曲线，当前仿真与手套采用端点之间的线性近似，拇指 IP/MCP 为 71/70，四指 DIP/PIP 为 78/99。手册示意图的拇指轴排列顺序与 URDF 串联顺序不同；项目按运动平面及此前 Qt 的逐轴观察绑定。

2026-11-08，操作者在本机右手 L20 上分别拖动两个 MuJoCo 滑块，确认 roll、yaw 的实物运动平面均与手册和仿真一致。以此表作为本机拇指通道对应依据。此项验证覆盖动作对应，角度行程采用手册规格。

Control 显示手册名称：`thumb_cmc_roll` 发送 SDK `thumb_roll`；`thumb_cmc_yaw` 发送 SDK `thumb_rotation`。其余滑块使用手指前缀和手册关节名，如 `index_mcp_pitch` 发送 `index_root`，`index_pip` 发送 `index_tip`。下列操作中的简称是 SDK 通道名。

按以下顺序逐个检查，每一步结束把刚动的滑块归回起点，避免叠加动作：

1. **建立零姿态**：所有滑块设 0，包括侧摆。每次只改一个滑块，从侧面观察手指；CMC/根部带动后续整段手指旋转，不代表那些后续关节自身的角度也变了。
2. **拇指末端**：`thumb_tip` 从 0 拖到 0.5，再到 1.0。MCP 和末节应同时弯曲；模型规定 `thumb_dip = (71/70) × thumb_mcp`，所以 MCP 为 0.5 时末节约 0.507 rad，MCP 为 1.0 时末节约 1.014 rad。末节没有独立滑块。
3. **拇指 CMC 三轴**：`thumb_roll` 先从 0 到 0.3，观察拇指根部转动；保持 roll 为 0.3，再分别检查 `thumb_rotation` 的 0→0.4→0，以及 `thumb_root` 的 0→0.3→0。检查后全部归零。零姿态附近部分 CMC 轴近似重合，两个滑块的外观运动可能相似，不能仅凭这一点判定映射重复。
4. **食指根部**：`index_root` 从 0 到 0.5，再到 1.0。整根食指从掌指关节弯曲，但 PIP/DIP 不应出现额外相对弯曲；归零。
5. **食指末端联动**：`index_tip` 从 0 到 0.5、1.0、1.57。只观察中节与末节的相对弯曲：`index_dip = 78/99 × index_pip`，对应 DIP 约 0.394、0.788、1.237 rad。根部目标仍为 0；归零。
6. **食指侧摆**：`index_abd` 从 0 到 −0.10、+0.10，再回 0。应主要横向摆动，不额外增加 PIP/DIP 弯曲。按当前右手模型，正角对应 raw 减小，负角对应 raw 增大。
7. **其余三指**：将上面三步的 `index` 分别换成 `middle / ring / pinky`。四指使用同一结构和 `78/99` 联动比例，但各自独立控制。

物理仿真使用位置伺服与 equality 约束，实际姿态会有跟踪过程及小误差。MuJoCo Control 显示的是目标值；不要把某帧可见缝隙或跟踪误差当作比例被改变。

## 怎样阅读和调用 SDK

先分清三层：`drivers` 是官方 SDK；`src/teleop/hand.py` 和 `glove.py` 是项目的调用封装；`mapping.py / retarget.py` 是我们自己的换算和跨手重定向，SDK 不会自动替我们完成这一步。

### Wuji SDK：提供采集、解算、标定等接口

| 用途 | 接口 | 本项目如何使用 |
|---|---|---|
| 管理与发现设备 | `SdkManager.instance()`、`scan()` | `glove.py` 筛选 WujiGlove |
| 建立/关闭连接 | `manager.connect(sn=..., device_name=...)`、`disconnect(name)` | `open_glove()` 上下文统一管理 |
| 查询左右手 | `glove.hand_side().get()` | 当前检查为右手 |
| 订阅关节角 | `glove.hand_joint_angles().subscribe()`、`sub.recv()` | 得到 5×5 槽、21 个有效角度及置信度，主遥操输入 |
| 骨架与指尖姿态 | `hand_skeleton().subscribe()`、`tip_poses().subscribe()` | SDK 已提供；当前角度映射不使用这两路 |
| 原始 EMF 位姿 | `emf_poses().subscribe()` | 官方离线测试的输入；不是 L20 命令 |
| 触觉与 IMU | `tactile()`、`tactile_zones()`、`imu_palm()`、`imu_data_palm()` 等资源 | SDK 已提供；当前手指控制不使用 |
| 手型标定与模型 | `calibrate_blocking(...)`、`hand_model_path().get()/set(...)` | 官方接口；当前映射没有自动启动标定 |
| 不接设备的离线解算 | `WujiGlove.offline_pipeline(...)`、`pipeline.hand_joint_angles().compute(emf)` | 全手测试实际调用的官方算法 |

`recv()` 非阻塞，没有新数据返回 None，不应把 None 当成零角度。项目的 `latest_angles()` 排空队列，只取最新帧。

接口位置：

- 项目连接/取帧封装：[`src/teleop/glove.py`](src/teleop/glove.py)。
- 官方用法与示例：[`drivers/wuji-sdk/examples/python/wuji_glove/`](drivers/wuji-sdk/examples/python/wuji_glove/)。
- 已安装的接口签名：`.venv/lib/python3.12/site-packages/wuji_sdk/__init__.pyi`，其中 `SdkManager`、`WujiGlove` 和各资源类型定义方法与参数。
- 真正运行实现：同目录 `wuji_sdk.cpython-312-x86_64-linux-gnu.so`。`__init__.py` 只是导入这个编译模块；`.pyi` 是接口声明，不是实现源码。原安装包保存在 `drivers/wheels/wuji_sdk-2026.10.26-...whl`。

使用项目封装读取一帧（示例需手套接入，本文整理过程未连接设备）：

```python
from teleop.glove import open_glove, latest_angles

with open_glove() as glove:
    sub = glove.hand_joint_angles().subscribe()
    frame = latest_angles(sub)
    if frame is not None:
        print(frame.fingers[1].angles)  # 食指，5 槽，最后一槽是 0 占位
```

### 新版 LinkerHand SDK：当前 L20 实际支持的接口

| 用途 | 接口 | 说明 |
|---|---|---|
| 发现并创建对象 | `discover_hands().require_one().create_hand_device()` | 按协议识别，项目检查 L20 right |
| 生命周期 | `hand.open()`、`hand.close()` | 通信连接，不等于电机上电/断电 |
| 身份与能力 | `get_device_info(timeout=...)`、`get_capabilities()` | 型号、SN、左右手、通道名、范围及支持操作 |
| 读取位置 | `get_angles(timeout=...).raw` | 16 个 raw，按 `joint_names` 排列 |
| 发送位置 | `set_angles(full_pose).wait(timeout=...)` | 完整 16 通道 raw 数组；本项目用 `write_named()` 排序 |
| 百分比位置 | `set_percent_angles(full_pose)` | SDK 提供 raw/百分比端点转换，不是弧度标定 |
| 速度 | `set_speeds(...)`、`get_speeds(timeout=...)` | 设置速度与实时速度读取，物理尺度需另核对 |
| 电流 | `set_current_limits(...)`、`get_currents(timeout=...)` | 前者是限幅设置，后者是实时电流，不能混为同一数值 |
| 状态与触觉 | `get_temperatures(...)`、`get_faults(...)`、`get_force(...)` | 温度、故障和触觉矩阵 |
| 清故障 | `clear_faults()` | 当前 L20 capabilities 声明支持 |

通用 SDK 虽然还定义了 `enable/disable/calibrate/save_parameters` 等方法，但当前 L20 profile 将这些操作标为不支持，不能仅看见方法名就认为这只手能用。以 capabilities 为准。

接口位置：

- 项目调用封装：[`src/teleop/hand.py`](src/teleop/hand.py)，包括 `open_hand / read_named / write_channel / write_named`。
- 官方公共设备接口：[`drivers/linkerbot-hand-python-sdk/src/linkerbot_hand/api/device.py`](drivers/linkerbot-hand-python-sdk/src/linkerbot_hand/api/device.py)；连接与 capabilities 在同目录 `_device_core.py`。
- L20 通道、范围、方向与支持能力：[`drivers/linkerbot-hand-python-sdk/src/linkerbot_hand/capabilities/models/l20.py`](drivers/linkerbot-hand-python-sdk/src/linkerbot_hand/capabilities/models/l20.py)。
- 设备发现：`drivers/linkerbot-hand-python-sdk/src/linkerbot_hand/discovery/`。
- L20 协议编解码与事务：`drivers/linkerbot-hand-python-sdk/src/linkerbot_hand/protocols/l20/` 的 `codec.py / transactions.py / identity.py`。
- CAN 传输实现：`drivers/linkerbot-transport/src/linkerbot_transport/can/`；设备运行时位于 `linkerbot_hand/runtime/`。

按 `hand.py → glove.py → mapping.py → cli.py` 阅读，再对照项目内官方文档与示例。

LinkerHand 最小读取流程：

```python
from linkerbot_hand import discover_hands

hand = discover_hands().require_one().create_hand_device()
try:
    hand.open()
    info = hand.get_device_info(timeout=1.0)
    names = hand.get_capabilities().joint_names
    sample = hand.get_angles(timeout=1.0)
    print(info, dict(zip(names, sample.raw, strict=True)))
    # 按 names 顺序准备完整目标数组后写入：
    # hand.set_angles(target).wait(timeout=1.0)
finally:
    hand.close()
```

重点理解发现设备、打开/关闭、能力与通道名、状态采样、完整目标数组、wait 和超时。`sample.raw` 是设备原始数值，不是弧度；16 通道顺序取自 `joint_names`，不要用 Qt 的 20 槽索引替代。官方说明在 `drivers/linkerbot-hand-python-sdk/docs`，传输实现位于 `drivers/linkerbot-transport/src`。

Wuji 对照 `glove.py` 与 `drivers/wuji-sdk` 官方示例，重点看 `SdkManager.instance()`、`scan()`、按序列号 `connect()`、`hand_side().get()`、`hand_joint_angles().subscribe()`、`recv()`、`disconnect()`。确认角度单位、分量语义、置信度及数据是否持续更新后，再定义映射。

Python SDK 直接提供设备调用；ROS SDK 额外提供节点、话题和消息组织。当前采集、映射、控制链使用 Python，不需要安装 ROS。

## 两条控制链路和独立 Qt 调试

两条正式控制链路共用 `src/teleop/hand.py`，均直接调用项目内的新版 SDK：

1. **手套遥操**：Wuji 手套 → `glove.py` 采集 → `retarget.py` 全手重定向 → `mapping.py` raw 转换 → `hand.py` → 新版 L20 SDK → USB-CAN → 真机。入口为 `python -m teleop follow-all --live`，输出全部 16 个主动通道。原 `follow` 保留作单通道验证。
2. **仿真滑块控制**：MuJoCo Control 滑块 → `sim_to_hand.py` 弧度转 raw → 同一个 `hand.py` → 同一个新版 L20 SDK → USB-CAN → 真机。入口为 `python scripts/sim_to_hand.py`，控制 16 个主动通道。

两条链路均不需要经过 Qt。手动可视化调试也可以直接使用新版 SDK：项目的 `qt.py` 已复用原官方界面的布局并替换设备调用，位置同样经过 `hand.write_named()`。旧版 SDK 目录已移入回收站，不再保留旧 Qt 启动入口。每次只运行一个真机控制入口。

纯仿真，不发送真机动作：

```bash
python scripts/view_l20_mjlab.py
```

展开 MuJoCo 的 Control，控件使用手册关节名（带手指前缀），控制量为弧度。项目内官方 L20 URDF 有 16 个主动、5 个联动关节；当前食指 PIP 上限约 `1.728 rad`，DIP 跟随 PIP，比例 `78/99`。

新版 SDK 的 Qt 调试界面：

```bash
QT_QPA_PLATFORM=wayland python -m teleop.qt
```

这是基于旧版官方布局迁移的项目界面，**不是厂家发布了新版 Qt**。显示 L20 right、16 个有效命名通道，没有旧协议的 4 个预留滑块；位置、速度、电流限幅和触觉读取均调用新版 SDK。滑块 raw 范围取 SDK capabilities，不使用 URDF 角度。原来的“扭矩”实际改接 `set_current_limits()`，因此更名为“电流限幅”，数值不是 N·m。动作预设来自旧 Qt 已有的 raw 姿态，按通道名称重排为 16 个值，不是角度限位表，也不是新版 SDK 提供的预设。启动用 SDK 回读姿态初始化控件；沿用原界面的周期发送及循环预设操作，没有新增操作按钮。“停止所有动作”沿用原实现，只停止预设循环，不是电机断电或急停。

桌面的“L20右手调试”快捷方式现在打开 `python -m teleop.qt`，使用新版 SDK 识别 L20 right。`scripts/start_qt.ps1` 负责 Windows/WSL 启动，`start_qt.sh` 进入 teleop 根目录并启动这个模块；它们不再引用旧 SDK。本文更新没有启动设备或下发动作。

MuJoCo 直接控制真机，需在现场运行：

```bash
python scripts/sim_to_hand.py
```

启动时通过新版 SDK 读取真机 raw 姿态，初始化仿真控件和联动关节；不因启动而发送目标。展开 Control 后拖动滑块，目标改变时调用 `hand.write_named()`，由 SDK 的 `joint_names` 决定完整数组顺序。关闭窗口结束发送并关闭设备连接。

参数来源：真机通道顺序、raw 范围和百分比端点语义来自新版 SDK 的 capabilities；L20 当前为 `0–255`，屈伸通道的 `0` 表示弯曲、`255` 表示伸直。新版 SDK 没有 L20 物理弧度映射表，因此仿真角度上下限采用手册行程，真机换算按手册行程与 SDK 端点线性对应。全手与仿真共用 `mapping.raw_endpoints()`，是本项目对模型和 SDK 动作语义的关联，不是 SDK 提供的弧度标定。

默认模式使用手册 V1.4 行程换算。直连对照模式可运行 `python scripts/sim_to_hand.py --legacy-ranges` 使用[旧 SDK 右手表](https://github.com/linkerbotai/linker_hand_sdk/blob/main/doc/Topic-Reference.md)，滑块采用手册行程，真机由新版 SDK 下发，超过旧表的角度会饱和。启动时保留真机反馈，拖动时仅更新被拖动通道的目标。不加参数即恢复默认换算。这次统一的是软件映射来源，实际物理角度仍需实物验证。

原 `sim_to_official_qt.py` 已移除，使用上述直接下发入口。该仿真入口不创建 Qt 界面，也不增加日志或录制功能。旧版 SDK 与原始 Qt 已移入回收站，桌面快捷方式改为新版 SDK 的项目 Qt。迁移后的界面与预设数据已在 `src/teleop/qt.py` 内独立保存，不依赖旧目录。

### 所有关节的参数来源与差异

真机的 16 通道顺序与原始值上下限来自新版 SDK；角度/raw 使用手册行程与项目名义端点对应；模型轴和安装位姿来自官方 URDF，工作限位采用手册值；两者之间的线性对应是本项目代码。手套单通道入口的伸直/弯曲端点则由用户实测输入，不能说已由 SDK 自动设置。

新版 SDK 没有另一份 L20 弧度表，因此无法计算“新版 SDK 弧度范围与 URDF”的差值。下面对比的是**以前使用的旧 SDK 换算表**与**目前使用的 URDF**，单位为 rad：

| 通道 | 旧 SDK 换算表 | 当前官方 URDF | 变化 |
|---|---|---|---|
| thumb_rotation | 0–1.78 | 0–1.22 | 上限减少 0.56，约 32° |
| thumb_roll | −0.297–0.683 | 0–1.40 | 零位和跨度都不同 |
| thumb_root | 0–0.87 | 0–0.79 | 上限减少 0.08，约 4.6° |
| thumb_tip | 0–1.29 | 0–1.05 | 上限减少 0.24，约 13.8° |
| 四指 root | 0–1.40 | 0–1.40 | 范围相同 |
| 四指 abd | −0.26–0.26 | −0.17–0.17 | 每侧少约 5.2° |
| 四指 tip（对应 PIP） | 0–1.08 | 0–1.57 | 上限增加 0.49，约 28.1° |

上表列出旧 SDK 和原始 URDF 的历史差异，当前运行范围以手册行程表为准。

模型限位、设备 raw 范围、自行定义的映射是三类信息。目录集成不会自动消除它们的差异。本次迁移不主动发送真机动作，已有真机功能需迁移后再现场复验。

## 用本项目理解工程术语

这些词是在描述“几段代码怎样可靠地一起工作”，不要求先学习一整套架构。阅读时先记住：谁产生什么数据、单位是什么、谁连接设备、谁负责关闭。

### 接口与公共契约

**接口**就是代码留给别人调用的入口。例如 `read_named(hand)` 读取状态，`write_named(hand, target)` 写完整目标。模块可以先理解为一个 `.py` 文件，函数是里面的一项操作。

**公共契约**是调用双方必须遵守的约定，比函数名和参数表更完整。“公共”是供其他模块使用，不是指互联网公开或要求你发布代码。以 `write_named()` 为例：

| 约定 | 当前项目的具体要求 |
|---|---|
| 对象 | 已打开的 L20 right 设备 |
| 数据内容 | 恰好包含 capabilities 给出的全部 16 个通道名 |
| 单位与类型 | 设备 raw 整数 0–255，不是 rad、百分比或 N·m |
| 顺序 | 调用者传命名字典，封装按 SDK 的 `joint_names` 排成数组 |
| 完成含义 | SDK 命令交付阶段完成，不代表手指已经到达目标 |
| 出错方式 | 缺通道、非法值、通信失败或超时等通过异常报告 |

例如“只发一个 `index_tip` 键”不满足完整目标契约。项目的 `write_channel()` 会先复制已有的完整姿态，只改其中一个通道，再调用 `write_named()`。把 MuJoCo 的 `0.5 rad` 直接当成 raw 发给 SDK，同样违反单位约定；必须先经过 `mapping.py`。

接口回答“怎么调用”，契约还回答“怎样调用才是对的、返回代表什么、失败怎么办”。契约清楚后，上层就不必研究每个 CAN 字节。

### 生命周期管理

**生命周期**是一个对象/资源从创建到释放的全过程。设备对象的典型过程是：

```text
发现设备 → 创建对象 → 打开通信 → 读状态/发命令 → 结束使用 → 关闭连接
```

这里 `hand.open()` 是打开通信，不是把机械手张开；`hand.close()` 是释放通信资源，不是给电机断电。关闭后不能继续把这个对象当作正常连接使用。

`hand.py` 的 `open_hand()` 已经把这套步骤组织好：打开设备，检查型号和左右手，把对象交给调用者；调用者结束或出错后，最终执行 `hand.close()`。

```python
from teleop.hand import open_hand, read_named

with open_hand() as hand:
    pose = read_named(hand)
    print(pose)
# 正常退出或块内抛出异常，都会进入封装的 finally，尝试关闭连接。
```

`with` 可以先理解为“进入时准备资源，离开时负责收尾”。实现它的 `@contextmanager`、`yield` 与 `finally` 都在项目的 `hand.py` 中：`yield` 前是准备，`yield` 后/`finally` 是收尾。程序被强制杀死、系统断电等情况不能依靠这段 Python 收尾来保证。

手套对应 `open_glove()`：连接指定手套，使用结束时断开这只设备。**资源所有权**就是“谁负责收尾”：在这里，连接封装负责设备连接，调用它的模块不应另起一套重复打开/关闭逻辑。重定向算法不负责连接或关闭设备。

### 封装、适配与依赖

**封装/适配**是在底层接口外加一小层符合本项目习惯的调用方式。`hand.py` 把 SDK 数组变成命名字典；`glove.py` 把设备筛选、右手检查和取最新帧组织起来。这不等于重写了官方驱动。

**依赖**是“这段代码运行需要谁”。目前调用关系是：

```text
手套入口 → glove.py → Wuji SDK
手套帧 → retarget.py → mapping.py → L20 raw 目标
手套/仿真/Qt 的真机位置命令 → hand.py → 新版 L20 SDK → CAN 传输
```

上层使用下层的稳定入口；官方 SDK 不应该反过来导入本项目的 Qt 或遥操代码。这样修改 Qt 布局不会要求修改 CAN 编解码，替换输入设备也不用重写机械手驱动。现在没有为这些概念额外添加接口类或新框架。

### 同步、异步、回读与超时

**同步**可以理解为“等这次操作给出结果再往下执行”；**异步**是“先提交，稍后观察结果”。新版 SDK 的 `set_angles()` 返回 `CommandHandle`，随后 `.wait(timeout=1.0)` 等待命令交付阶段完成。

SDK 在 `api/commands.py` 明确区分 accepted、dispatched、acknowledged 等阶段。对于没有协议确认的命令，成功交给传输层就可能是完成状态；即便收到确认，也不代表回值等于目标，更不代表机械动作已经完成。因此“调用没报错”“收到状态”“实物到位”是三个不同结论。

**回读**是再用 `get_angles()` 查询设备当前反馈，与目标比较。`write_named()` 不会替调用者证明机械到位。**超时**表示规定时间内没有等到所需结果；它不是“设备一定没收到”或“电机已停止”。

手套的 `sub.recv()` 又是非阻塞取帧：没新数据就返回 None。None 不代表所有关节角为零。`latest_angles()` 取最新帧，避免动作已经变化、程序却在处理队列中的旧姿态；这属于**数据新鲜度**的约定。

### capabilities：先问这只设备支持什么

可以把 capabilities 理解成“这只设备的能力说明书”。`get_capabilities()` 给出通道名、顺序、原始范围和支持的操作。

SDK 的统一设备类可能有很多方法，但不代表每种机械手都支持。当前 L20 声明支持位置、速度、电流限幅、相关状态读取及清故障；`enable/disable/calibrate/save_parameters` 等操作则未声明支持。这就是为什么不能凭方法名推断设备行为。

建议从 `hand.py` 的 `open_hand → read_named → write_named` 阅读上述概念，再看 `glove.py`，最后看 `retarget.py` 与 `cli.py`。先理解数据和资源如何流动，再深入 SDK 的 runtime、协议和 CAN 实现。

## 本次集成验证

MuJoCo 直连真机入口 `scripts/sim_to_hand.py` 仅用于姿态与方向对照，关闭接触碰撞。官方 URDF 导入后的碰撞网格存在结构重叠：此前测试模型轴 `thumb_cmc_yaw` 时，目标为 0、0.7、1.4 rad 时，开启接触的仿真实际角停在约 1.39–1.43 rad；关闭接触后分别到达约 0、0.70、1.40 rad。该入口因此不用于接触、抓取或训练验证；未修改官方资产及其他仿真入口。后续 hora 适配需单独处理碰撞几何。

已撤回按相机观察设置的五通道方向覆盖。当前手套与 MuJoCo 直连共用手册行程换算。真机 raw 和百分比端点含义以新版 SDK 的 L20 capabilities 为准。

此前已将核心项目复制到独立临时目录并重新安装，确认 SDK 均从副本内部导入，命令帮助正常。新版官方 SDK 的 1628 项无硬件测试通过。项目内 URDF 加载、RTX 4060 上的 mjlab 仿真步进及依赖兼容性检查通过。该独立副本检查阶段没有执行真机运动；后续实机方向验证结果见本节开头。

MuJoCo 直连新版 SDK 的改动已用真实 GPU 仿真、模拟设备接口检查：16 通道 raw/弧度往返、启动不写入、仅食指末端目标往返、其他通道不变、按 SDK 通道顺序发送和关闭连接均通过；运行过程没有导入旧 SDK 或 Qt。手套单通道写入也已检查共用的 `write_named()`。

迁移后的 Qt 已用模拟设备完成离屏渲染与调用检查：16 通道和标签、食指独立目标、预设顺序、16 通道速度/电流限幅、触觉矩阵及关闭连接均通过，没有导入旧 SDK。未连接真实设备启动或下发动作。

全手映射及 16 通道方向回归共 25 项测试通过，包括官方 Wuji 离线解算器实际输出帧。另以 100 帧模拟全手角度检查越界和 CMC 求解，本机平均约 5.5 ms/帧；这只是映射计算时间，不包含 USB/CAN 通信。手套尚未接入实测。

## RM75 arm、L20 hand 与联合 MuJoCo 调试

已将 arm 与 hand 的设备调用和联合模型纳入本项目，三个独立入口为：

~~~bash
cd '/mnt/d/data/learning/EMG+灵巧手/teleop'
.venv/bin/teleop-arm
.venv/bin/teleop-hand
.venv/bin/teleop-arm-hand
~~~

也可以使用 .venv/bin/python -m teleop.mujoco_control arm、hand 或 arm-hand。每个入口可加 --preview 仅显示模型，可加 --check 仅编译并打印滑块信息，两者均不连接硬件。

在已有 hand 环境中安装 arm SDK 和新入口使用：

~~~bash
uv pip install --python .venv/bin/python --no-deps drivers/wheels/robotic_arm-1.1.7-py3-none-any.whl
uv pip install --python .venv/bin/python --no-build-isolation --no-deps -e .
~~~

默认是实机调试。arm 模式只连接 RM75，hand 模式只连接 L20，arm-hand 模式连接两者。arm 界面会显示安装在腕部的手作为固定外观，但不连接手的设备。arm-hand 的 Control 内共 23 个滑块，ARM_J1_deg 至 ARM_J7_deg 使用度，L20_*_rad 的 16 个主动通道使用弧度。

启动时以设备当前读数初始化滑块，不自动回零。手的初始 raw 值按现有 mapping 转换，并保留未改动通道的 raw 值，直到该通道的滑块被移动。滑块变化后，arm 直接调用 rm_movej_follow，hand 复用 hand.py 的 write_named。模型显示目标姿态，手的从动关节由原始 mimic 关系更新。

这是关节目标与安装关系的运动学预览，未做执行器动力学仿真。原有 scripts/sim_to_hand.py 和 view_l20_mjlab.py 继续保留 GPU 物理仿真路径。新界面不要求为 arm 的显示额外启动 mjlab/GPU 仿真。

### rm_movej 与 rm_movej_follow

rm_movej 为目标做点到点规划，v 是速度百分比，适合明确的离散目标。连续拖动时不断提交新点，可能滞后执行旧规划。rm_movej_follow 是在线规划跟随最新关节目标，适合滑块与遥操作不断更新的目标，不带 v 百分比参数，使用控制器的运动参数。它与严格周期的高频 CANFD 透传仍是不同接口。

当前 arm 使用 rm_movej_follow。旧版额外的 ±1° 窗口、0.5°/s 限速和心跳中止机制没有引入此入口。

### 连接、关闭与源码

实机联调在 WSL Ubuntu-22.04 的本项目 .venv 内运行。USB-CAN 的 can0 连接和配置沿用前面的 hand 环境；arm 使用 192.168.1.18:8080，可通过 --ip 修改。空间方向与设备端控制参数沿用真实硬件配置。

关闭界面后停止 arm、清空旧轨迹并断开设备连接，L20 SDK 会关闭。空格停止 arm 并清空其轨迹；手仍保持已下发姿态。单独手的旧入口继续按它原来的行为运行。

新增代码为 src/teleop/arm.py、mujoco_model.py、mujoco_control.py。RM75 原始 URDF、网格、许可证位于 models/RM75，SDK wheel 位于 drivers/wheels，因此当前入口不依赖同级的睿尔曼调试目录。

arm 的地面 base_link 经七个关节连到 Link7，再由固定关节连到 L20 的 l20_base_link。models/RM75/mount.json 保存近似安装参数，当前为零平移、绕 Z 轴 160°，仍属待测初值。两个 base_link 的原点处于不同位置。模型导入使用 rm75_ 与 l20_ 网格名，避免原始两份 base_link.STL 同名导致手掌误载入底座外形。


### 模型一致性检查

运行 .venv/bin/python -m unittest discover -s tests -v，检查两份 base_link 网格独立加载、全部手部网格与单手模型一致，以及手的内部运动学在固定安装后保持一致。

每台设备同一时刻使用一个调试入口，避免 arm 窗口或手套/Qt 写入程序与联合界面同时下发目标。

## 桌面参考姿态

profiles/table_facing.json 保存了 2026-11-08 17:20:51（北京时间）直接从实际设备读取的参考姿态，包括七轴角度、末端位姿、工作/工具坐标系、型号与软件版本、L20 命名通道 raw 值和当前近似安装变换。arm 和 hand 顺序读取，时间分别保存在文件中。

本次七轴角度约为 [173.932, -4.363, 1.403, -109.620, -1.811, -12.194, 29.532]°。末端 pose 为 [0.340237, -0.028999, 0.315265, -2.576, -0.797, -0.813]，前三项单位米，后三项单位弧度，使用文件记录的当前工作与工具坐标系。

该文件是用户指定的“面向桌面”参考读数。保存操作不发送运动指令，启动调试仍读取设备当前姿态。未增加按键保存功能，也未将参考姿态改写为控制器机械零位。

旧的独立睿尔曼调试目录和历史运行记录已清理；SDK、模型与运行代码统一在本项目内。当前只保留实际使用的三种调试入口及已有手套、Qt 和 GPU 仿真功能。

## RM 官方 SDK 仓库

官方仓库为 https://github.com/RealManRobot/RM_API2 ，本地在 drivers/realman-rm-api2，已固定官方 1.1.7 标签，对应提交 a810b9fdea29e3567c5e9155794e6ef9cc17be62。采用稀疏检出，展开 Python 目录和根目录说明；仓库同时包含 C/C++ SDK，可按需展开。

Python 包源码在 Python/Robotic_Arm/rm_robot_interface.py 与 rm_ctypes_wrap.py，原生库在同目录 libs。该仓库提供 Python 封装代码及厂商预编译库；运行时仍使用已安装的 Robotic_Arm 1.1.7，源码检出未切换运行环境。

已核对官方 1.1.7 标签与虚拟环境中的 Robotic_Arm 1.1.7。Python 文件统一换行后文本一致，原生库逐字节一致。Windows Git 的 CRLF 转换会使 Python 文件原始哈希不同，不代表接口实现版本不同。

### 本地 SDK 加载方式

当前按用户确认的本地方案保留 RM 的 wheel 安装方式。实际调用链为 src/teleop/arm.py → .venv/lib/python3.12/site-packages/Robotic_Arm → 包内 libs/linux_x86/libapi_c.so → RM75 控制器。drivers/realman-rm-api2/Python 保存对应版本的官方源码，修改该目录不会自动影响当前运行。

LinkerHand 和 transport 使用 editable 安装，实际加载各自在 drivers 中的代码。两种安装方式的差别是代码来源与修改生效方式；对目前已核对一致的 RM 1.1.7，切换到仓库源码不会改变接口或控制行为，也不会提高机械臂响应速度。

后续需要修改 RM Python 封装时，再切换为项目内源码安装，并核对导入路径及原生库路径。仅调整我们自己的连接或目标下发逻辑时，修改 src/teleop/arm.py 即可。

可在 WSL 项目目录查看实际导入位置：

~~~bash
.venv/bin/python -c "import Robotic_Arm.rm_robot_interface as sdk; print(sdk.__file__)"
~~~

# teleop
