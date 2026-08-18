# Simscape 串联腿导入

这里用于把 SolidWorks 串腿底盘装配体导出的 Simscape Multibody XML 导入 MATLAB，并逐步建立真实机构、关节驱动和轮地接触模型。

当前状态：SolidWorks 源文件已确认，MATLAB R2024b 的 Simscape Multibody 可用；还缺少从 SolidWorks 插件导出的 XML，因此暂时不能执行实际装配体导入。

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

## 重要边界

SolidWorks 装配体导入后只是机构的初始物理骨架。轮地接触、摩擦、执行器限幅、传感器噪声和六维控制器仍需在 Simulink/Simscape 中配置。
