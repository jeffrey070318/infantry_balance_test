# 运行顺序

在 MATLAB 桌面端命令窗口逐行运行：

```matlab
cd('D:\Project\RMproject\infantry_balance_test\matlab\serial_leg_6dof')
startup_serial_leg_6dof
result = run_serial_leg_6dof(true);
tests = runtests('tests');
table(tests)
```

`result.dynamics` 是哈工程参考参数下的六维非线性方程、线性化结果、LQR和腿长扫描；`result.mechanism` 是旧占位闭环的Jacobian和VMC接口检查；`result.cadTopology` 是本队STEP轴线、轴距和传动轴证据；`result.cadKinematics` 是J1两个关节输出角驱动的真实CAD闭环正解；`result.cadTaskSpace` 是轮心刚性附着在连杆4后的等效腿长、腿角和任务Jacobian；`result.actuators` 保存DM-J8009与M3508/C620手册参数及当前传动标定状态。

图10用于观察线性/非线性是否一致和增益调度误差；图11只展示占位几何的接口；图12展示本队CAD参考姿态；图13分别扰动两个输出角，检查七个机构节点是否按闭环约束运动；图14展示轮心任务空间和Jacobian条件数。当前 `q(1)`、`q(2)` 是J1处两个关节输出角，不是电机编码器角；轮心已由7个安装孔证据确认刚性附着在连杆4上，J7仍只是机构节点而不是轮轴。

第一次阅读建议依次打开 `heu_reference_parameters.m`、`equivalent_leg_dynamics_6dof.m`、`linearize_equivalent_leg_6dof.m`、`design_heu_lqr_6dof.m`。看懂六维状态如何从三条二阶方程变成 `A/B/K` 后，再看 `cad_serial_leg_parameters.m`、`cad_serial_leg_forward_kinematics.m` 和 `cad_serial_leg_node_jacobian.m`。旧的 `closed_chain_forward_kinematics.m` 只用于理解两圆交点与VMC接口，不代表本队机构。

当前不要修改 `Q/R` 追求更漂亮的曲线。先替换真实质量、惯量和几何参数并重新验证，再进行控制权重调参；否则得到的只是另一套参考机器人的 K。

## Simulink 最小演示

在 MATLAB 命令窗口运行：

```matlab
cd('D:\Project\RMproject\infantry_balance_test\matlab\serial_leg_6dof')
startup_serial_leg_6dof
result = run_simulink_6dof_demo(true);
```

脚本会生成并打开 `simulink/serial_leg_6dof_demo.slx`，同时打开俯仰角 Scope。模型使用19 kg、`L0=0.30 m`的线性六维对象和10°初始俯仰，只用于直观看LQR闭环回正效果。

要观看简化三维机器人随 Simulink 状态运动，运行：

```matlab
animation = run_simulink_3d_animation_6dof(true);
```

三维画面会同步显示两轮、两腿和机身俯仰，并在 `results/27_simulink_3d_motion.mp4` 保存视频。该画面由六维状态驱动，不包含轮地接触碰撞。
