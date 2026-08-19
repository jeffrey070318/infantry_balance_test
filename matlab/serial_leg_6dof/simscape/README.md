# Simscape 串联腿导入

这里用于把 SolidWorks 串腿底盘装配体导出的 Simscape Multibody XML 导入 MATLAB，并逐步建立真实机构、关节驱动和轮地接触模型。

当前状态：完整底盘和右腿都已实际导出。完整底盘模型过大且配合冲突较多；单独导出的 `腿A.SLDASM` 已成功生成 Simscape 模型，删除 6 个由重复约束产生的无效关节后能够编译，并完成 `0.05 s` 短时求解。

## 当前对应的 SolidWorks 装配体

```text
D:\Project\SWproject\轮腿\串腿底盘2.8\串腿底盘.SLDASM
```

## 导入顺序

1. 在 SolidWorks 中打开 `串腿底盘.SLDASM`。
2. 使用 Simscape Multibody Link 导出 XML 和关联几何文件。
3. 将 XML 放入本目录下的 `import/` 文件夹。
4. 在 MATLAB 中运行：

```matlab
cd('D:\Project\RMproject\infantry_balance_test')
startup_serial_leg_6dof
result = import_serial_leg_cad_simscape;
```

5. 先检查导入后的刚体、关节和坐标轴，不要立即加入复杂接触。
6. 再依次加入关节驱动、关节传感器、轮子和地面接触。

## 当前右腿模型

右腿导出文件放在本地 `right_leg_import/` 中。重新生成并验证清理后的模型：

```matlab
cd('D:\Project\RMproject\infantry_balance_test')
startup_serial_leg_6dof
result = build_sanitized_right_leg_simscape(true);
```

脚本会保留原始导入模型，并另外生成 `serial_leg_right_cad_sanitized.slx`。当前清理只删除 Simscape 编译器明确报告“两端已被刚性连接”的 6 个冗余关节，不代表关节拓扑已经适合接入控制器。下一步仍需识别真正的髋、膝和轮轴自由度，并删除轴承、螺钉等产生的无关运动副。

## 重要边界

SolidWorks 装配体导入后只是机构的初始物理骨架。轮地接触、摩擦、执行器限幅、传感器噪声和六维控制器仍需在 Simulink/Simscape 中配置。
