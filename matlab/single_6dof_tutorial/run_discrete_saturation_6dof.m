function result = run_discrete_saturation_6dof(showPlots)
%RUN_DISCRETE_SATURATION_6DOF Add MCU sampling and actuator saturation.

% This is still the fixed-leg-length six-state reference plant. The new
% layer models how firmware updates the command periodically and how an
% actuator clips a command that exceeds its available torque.

arguments
    showPlots (1, 1) logical = true
end

model = paper_model_6dof();
controller = design_lqr_6dof(model);
sampleTime = 0.001;
t = (0:sampleTime:10).';
reference = limited_speed_reference_6dof(t, 1.5, 3.0, 0.7);

adequate = simulate_sampled_tracking_6dof( ...
    model, controller, reference, [Inf; Inf]);
% Teaching limits only. These are not selected from a real motor system.
torqueLimits = [2.5; 1.0];
insufficient = simulate_sampled_tracking_6dof( ...
    model, controller, reference, torqueLimits);

fprintf('\n=== 六维模型：1 kHz 离散控制与力矩饱和 ===\n');
fprintf('控制周期:                  %.3f ms\n', sampleTime * 1000);
fprintf('轮矩限制 |T|:              %.3f N*m\n', torqueLimits(1));
fprintf('等效摆矩限制 |Tp|:         %.3f N*m\n', torqueLimits(2));
fprintf('发生任一输出饱和的采样占比: %.3f %%\n', ...
    100 * insufficient.saturationFraction);
fprintf('正常控制所请求的最大 |T|:  %.3f N*m\n', ...
    adequate.maxRequestedTorque(1));
fprintf('正常控制所请求的最大 |Tp|: %.3f N*m\n', ...
    adequate.maxRequestedTorque(2));
fprintf('未限幅最大腿角:            %.3f deg\n', ...
    rad2deg(adequate.maxLegAngle));
if insufficient.isDiverged
    fprintf('限矩不足判定:              已发散\n');
    fprintf('首次超过状态阈值的时刻:    %.3f s\n\n', ...
        insufficient.divergenceTime);
else
    fprintf('限矩不足判定:              未发散\n');
    fprintf('限幅后 10 s 位置误差:      %.6f m\n', ...
        insufficient.finalPositionError);
    fprintf('限幅后 10 s 速度误差:      %.6f m/s\n\n', ...
        insufficient.finalVelocityError);
end
fprintf(['注意: 2.5/1.0 N*m 是演示饱和机制的假设值，不能作为' ...
    '实际电机或关节限矩。\n\n']);

if showPlots
    figureHandle = plotDiscreteComparison(reference, adequate, insufficient);
    result.figurePath = save_figure_6dof( ...
        figureHandle, "04_discrete_control_and_saturation");
else
    result.figurePath = "";
end

result.model = model;
result.K = controller.K;
result.reference = reference;
result.adequate = adequate;
result.insufficient = insufficient;
end

function figureHandle = plotDiscreteComparison(reference, adequate, insufficient)
figureHandle = figure('Name', '离散控制与力矩饱和', 'Color', 'w');
tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(reference.t, reference.velocity, ':', ...
    adequate.t, adequate.x(:, 4), '--', ...
    insufficient.t, insufficient.x(:, 4), 'LineWidth', 1.2);
grid on;
ylabel('速度 (m/s)');
legend('期望', '1 kHz 未限幅', '1 kHz 已限幅', 'Location', 'best');
title('固定周期计算：u[k] 在相邻采样点之间保持不变');

nexttile;
plot(adequate.t, rad2deg(adequate.x(:, 1)), '--', ...
    insufficient.t, rad2deg(insufficient.x(:, 1)), 'LineWidth', 1.2);
yline(15, ':', '暂定小角度边界');
grid on;
ylabel('腿角 theta (deg)');
legend('未限幅', '已限幅', 'Location', 'best');

nexttile;
plot(adequate.t, adequate.u(:, 1), '--', ...
    insufficient.t, insufficient.u(:, 1), 'LineWidth', 1.2);
yline(insufficient.torqueLimits(1), ':');
yline(-insufficient.torqueLimits(1), ':');
grid on;
ylabel('实际轮矩 T (N*m)');
legend('未限幅', '已限幅', 'Location', 'best');

nexttile;
plot(insufficient.t, insufficient.uRaw(:, 1), '--', ...
    insufficient.t, insufficient.u(:, 1), 'LineWidth', 1.2);
grid on;
xlabel('时间 (s)');
ylabel('轮矩指令 T (N*m)');
legend('限幅前计算值', '限幅后执行值', 'Location', 'best');
end
