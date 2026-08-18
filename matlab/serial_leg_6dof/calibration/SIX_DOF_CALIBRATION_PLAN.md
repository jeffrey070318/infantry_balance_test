# 六维实机标定计划

标定顺序必须是“断电机械确认 -> 低功率方向确认 -> 零位记录 -> IMU符号 -> 轮侧方向/效率”。在零位和方向没有确认前，不运行闭环平衡，不把仿真 K 直接发给电机。

## 1. 机械参考姿态

把左右腿都放到 CAD 参考姿态：任务腿角 `0 deg`、等效腿长 `L0=0.306802737 m`。MATLAB 对应的两个机构输出角约为：

```text
q1 = 38.174 deg
q2 = 141.823 deg
```

记录四个 DM8009 的原始位置、左右侧和电机 ID。这个姿态只定义机构输出参考，不代表电机编码器零位已经确定。

## 2. DM8009 方向与零位

每次只使能一个关节，使用很小的 MIT 位置/力矩指令，确认“电机角增加”时对应的机构输出角是增加还是减少，得到 `rotationSign=[s1;s2]`，每个值只能是 `+1` 或 `-1`。随后在机械参考姿态记录 `motorZeroRad`，并把对应机构输出参考写入 `outputZeroRad`。

接口公式固定为：

```text
outputAngle = outputZeroRad + rotationSign .* (motorAngle - motorZeroRad)
```

需要左右两侧分别记录，不能假设左右镜像后符号自然相同。

## 3. IMU 安装变换

机器人断电静置，分别记录右侧和左侧代码实际使用的 `Pitch`、`Gyro[Y]`。以“机体鼻端上抬”为物理正方向，验证：

- 右侧 `myPithR` 与 `INS.Pitch` 的符号；
- 左侧 `myPithL` 与 `-INS.Pitch` 的符号；
- 左右 `myPithGyro*` 与 `Gyro[Y]` 的符号；
- 静止时 `phi_dot` 是否接近 `0 rad/s`。

固件源码当前明确存在左右侧相反的 Pitch/Gyro 符号，不能用单侧实测结果替代另一侧。

## 4. 轮侧方向与减速器

悬空状态下单侧低速点动，记录电机编码器正方向、轮轴正方向和轮边齿轮输出方向。再用已知电机转数测轮轴转数，确认总减速比，而不是只使用 P19 内部 `3591/187`。效率先记录为未知，不能把无损耗仿真上限当作实机连续转矩。

## 5. 原始数据字段

每条记录至少包含：`timestamp_s, side, motor_id, motor_angle_rad, output_angle_rad, motor_velocity_rad_s, output_velocity_rad_s, torque_command_Nm, measured_current_A, imu_pitch_rad, imu_gyro_y_rad, wheel_speed_rad_s, wheel_direction_sign`。

原始数据不覆盖，标定结果另存；每次标定记录电池电压、软件版本、机械状态和操作者。

## 6. 通过条件

- 四个关节方向重复三次结果一致；零位回到参考姿态的输出角误差小于 `1 deg`。
- 左右 IMU 静止偏置和符号记录完整，`phi_dot` 静止 RMS 小于 `0.02 rad/s`。
- 轮侧方向与总减速比在正转/反转重复测量中一致。
- 只有以上条件满足，才把 `isCalibrated`、`isResolved` 从 `false` 改为 `true`，并重新运行六维全套测试。

填完记录后先运行：

```matlab
check = validate_calibration_record_6dof(record)
actuators = apply_calibration_record_6dof(record)
```

第二条命令只有在所有标定项和元数据均通过时才会执行；否则主动报错并列出阻塞项。
