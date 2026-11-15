# 来源与接口调整

Hand2 thumb_track_b4 算法、SDK 适配、配置和模型来自本仓库 d2d8bb2；核心源码保持原样，调用厂商二进制 wuji-sdk 2026.10.26。本地部分为连续表面对应与拇指四关节 SLSQP 约束优化；非学习模型、非触觉闭环。

模型许可证保留在 models/Wuji/LICENSE 和 models/Wuji/hand2_beta1/LICENSE。依赖发行包保留各自许可证；未为本地实现新增开源许可证。个人标定、录制和 SDK 二进制不发布。

项目名由 wuji-l20-teleop 改为 wuji-hand2-teleop；python -m teleop 和 hand2-teleop 进入 Hand2。移除 L20/Qt/联合手臂手入口。主线 RM75 的 arm.py SDK 调用保持原样；模型与控制入口仅保留独立机械臂，不再挂载 L20。hand2/thumb-track-b4 分支进一步移除机械臂全部实现和依赖。
