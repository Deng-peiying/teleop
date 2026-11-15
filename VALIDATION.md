# 验证（2026-11-15）

在既有 WSL Ubuntu-22.04 / Python 3.12 teleop-current 环境中执行，不连接硬件。

- 原 B4 核心 13 个运行模块 Git 内容全部保持不变（SHA256 以 Git blob 为准，忽略检出时 CRLF 转换），清单见 source_manifest.json。
- 主线 unittest discover：6 项全部通过，覆盖接触/释放、各指双向接触、无测量标定拒绝、SLSQP 舍入终止、录像编码及独立 RM75 七轴角度转换。
- python -m teleop --help 和独立 RM75 --check 检查入口和模型依赖。
- 纯 Hand2 分支移除机械臂测试后执行剩余 5 项回归测试，并进行实际构建 B4、调用 SDK、模型步进的 450 帧离线回放（51 帧启用接触修正）；用于本地验证的个人数据不提交。

未验证全新联网安装、未执行新的手套试戴或机械臂实机动作。
