# Wuji Glove → Wuji Hand2 左手仿真遥操

当前主方法为 `thumb_track_b4`。main 保留独立 RM75 调试；`hand2/thumb-track-b4` 分支仅保留手套到 Hand2 的仿真遥操。旧 LinkerHand L20 功能、模型和依赖已清理，历史版本可在 Git 历史查阅。

## 算法

每帧输入 Wuji SDK 骨架的 21 个位置（米）及 21 个四元数（XYZW）。官方二进制 SDK `RetargetSession.for_hand(HandModel.WujiHand2, Handedness.Left)` 生成 20 关节参考；本地 NLopt `LD_SLSQP` 使用解析梯度，仅优化拇指 4 个关节，其他 16 个关节保持 SDK 输出。

连续表面对应 `ContinuousSurfacePatch` 在网格局部 UV 坐标中追踪接触区域，使用源运动前馈、局部有界修正和有界切向历史。目标包含法向间隙、切向位置及增量、法向朝向、骨段几何和相对 SDK 修正量的连续性。关节范围、每帧速度界限和配置的拇指碰撞几何对构成约束；释放时回归 SDK，并处理食指回滑释放的第一轴反弹。

实际求解器参数在 `src/teleop/thumb_relative.py`：4 变量，最多 80 次评估，速度界限 3 rad/s，网格间隙目标 0.15 mm。继承的 `profiles/hand2_retarget_experiment.json` 原样保留，其中部分早期实验字段未被 B4 使用；不能把其中 `max_evaluations=24` 或 `max_velocity_rad_s=6` 当作 B4 实际参数。

接触权重来自骨架几何与估计皮肤厚度，不是传感器测得的接触。指腹朝向也是模型估计。本方法没有训练网络、没有触觉闭环；SDK 内部算法为厂商实现，不能归为本地原创。食指侧缘往返滑行曾经左手试戴认可，其他三指、物体抓握和真机性能未完成系统验收。0.15 mm 是模型约束目标，不是真实接触精度保证。

## 安装

使用 Linux/WSL Ubuntu、Python 3.12。在本目录运行：

```bash
python3.12 -m venv "$HOME/.venvs/teleop-current"
source "$HOME/.venvs/teleop-current/bin/activate"
python -m pip install -r requirements-hand2.txt
sudo apt install libegl1 ffmpeg
```

完整版本锁定保存在 `requirements-hand2.txt`：wuji-sdk 2026.10.26、pin 4.1.0、nlopt 2.11.0、mujoco 3.16.0、numpy 2.5.4、scipy 1.18.1。SDK 从厂商发行渠道取得；若当前索引不提供这些版本，应配置对应发行源，不能静默换版本。ffmpeg 仅录像需要。`pin` 是 Pinocchio 的 pip 包名。

## 使用

先通过厂商 SDK 配置并标定自己的左手用户。个人标定文件和账号不随包发布。默认用户名 `operator_left`，设备地址 `192.168.1.100:50001` 均可覆盖：

```bash
PYTHONPATH=src MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python -m teleop.hand2_live --method thumb_track_b4 \
  --user operator_left --address 192.168.1.100:50001 --display command
```

`command` 显示关节目标；`--display dynamics` 显示模型动力学响应。窗口固定手腕，不接入 Tracker。F9 开始/结束录像；退出时保存当前片段。记录写入本目录 `runs/hand2_live/<时间>/`。程序会临时选择 SDK 用户，结束后恢复此前用户；同一设备避免并行运行多个订阅程序。

Windows PowerShell 可在本目录执行 `./start.ps1`，它原样使用 WSL `Ubuntu-22.04` 和 `$HOME/.venvs/teleop-current`；自定义用户或地址时使用上面的 Python 命令。

无 GUI 的硬件检查也会连接真实手套，但不向机械手下发命令：

```bash
PYTHONPATH=src python -m teleop.hand2_live --headless --duration 5 --user YOUR_USER --address YOUR_ADDRESS
```

代码调用：用至少两帧有效初始骨架构建 `build_method('thumb_track_b4', samples, config, dt)`，然后调用 `method.step(keypoints, orientations)`。`samples` 为 N×21×3；`dt` 为秒；返回 20 个弧度目标，按拇指至小指、每指 4 关节的固件顺序。完整命名见 `firmware_joint_names('left')`。四元数顺序不可改成 WXYZ。可从 `method.side_diagnostics` 获取求解状态和模型间隙。模型、相对目录和包名 `teleop` 必须保留；使用独立环境避免与其他同名包冲突。

## 验证和来源

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
PYTHONPATH=src python -m teleop --help
```

B4 核心算法保留自本仓库 d2d8bb2，未修改目标函数、梯度、接触追踪、参数或模型。来源及验证范围见 PROVENANCE.md、VALIDATION.md。

## RM75 独立调试（仅 main）

保留官方模型、arm.py SDK 和七轴滑块，移除 L20 外观与联合控制。

```bash
python -m pip install '.[arm]'
PYTHONPATH=src python -m teleop.mujoco_control --check
PYTHONPATH=src python -m teleop.mujoco_control --preview
# 实机连接；启动停止并清空旧轨迹，然后读取当前关节
PYTHONPATH=src python -m teleop.mujoco_control --ip 192.168.1.18
```

机械臂默认 192.168.1.18:8080，滑块单位为度；变化时调用 rm_movej_follow，空格停止并清空轨迹，关闭停止、清轨迹并断开。Hand2 遥操不连接机械臂。本次未执行新的真机运动。
