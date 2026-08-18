# infantry_balance_test

RoboMaster 串联腿平衡步兵的建模与仿真工程。目前重点是单侧六维等效模型，包含 MATLAB/Simulink、LQR、串联腿闭环运动学、VMC、执行器边界和简化三维运动展示；十维模型暂未开始。

## 快速运行

```matlab
cd('matlab/serial_leg_6dof')
startup_serial_leg_6dof
result = run_serial_leg_6dof(true);
```

运行 Simulink 最小闭环和三维动画：

```matlab
result = run_simulink_6dof_demo(true);
animation = run_simulink_3d_animation_6dof(true);
```

详细说明见：

- `轮腿开发文档.md`
- `matlab/serial_leg_6dof/RUN_GUIDE.md`
- `matlab/serial_leg_6dof/MODEL_CONTRACT.md`
- `matlab/serial_leg_6dof/FIRMWARE_INTERFACE_6DOF.md`

## 当前边界

当前质量、质心和惯量包含工程暂估值，三维动画是六维状态驱动的可视化，不是完整的 Simscape Multibody 轮地接触仿真。仿真结果不能直接作为实机控制参数下发。

## 资料说明

`重要参考资料` 和部分 CAD 文件来自各自作者、战队或厂商，仅用于学习、复现和来源追踪；其著作权和授权条件仍归原权利人所有，本仓库不对这些第三方资料作重新授权。
