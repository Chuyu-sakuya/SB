6.14晚上创建
6.15完成openocd配置舵轮底盘理论学习第一版舵轮底盘代码完成
仅凭眼看就感觉有bug  后续修改
6.16
严肃复习C++
6.17
feat(chassis): 优化舵轮运动学解算，新增最小旋转角策略与零速角度保持

- 新增 DeltaAngle 最小旋转角逻辑：目标角差超过90°时自动反转，舵电机单次转动不超过90°
- 新增零速死区保护：四轮轮速均低于阈值时维持上一拍角度，避免静止抖动
- 变量语义重构：HeadingAngle 拆分为 GoalAngle/ExternAngle/LastAngle/DeltaAngle
- swerveWheelResolve 函数声明移至 Chassis_Task.h
- 清理 main.c 中冗余的 CAN 初始化片段
