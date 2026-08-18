function result = run_acceleration_sweep_6dof(showPlots)
%RUN_ACCELERATION_SWEEP_6DOF Find a teaching-range acceleration command.
%
% The model, LQR gain, target speed, and distance stay unchanged. Only the
% acceleration limit changes, so its effect on angle and torque is visible.

arguments
    showPlots (1, 1) logical = true
end

model = paper_model_6dof();
controller = design_lqr_6dof(model);
t = (0:0.005:10).';
accelerations = (0.5:0.1:1.0).';
n = numel(accelerations);

maxLegAngleDeg = zeros(n, 1);
maxBodyPitchDeg = zeros(n, 1);
maxWheelTorque = zeros(n, 1);
stopTime = zeros(n, 1);
withinSmallAngleRange = false(n, 1);

for index = 1:n
    reference = limited_speed_reference_6dof( ...
        t, 1.5, 3.0, accelerations(index));
    simulation = simulate_tracking_6dof(model, controller, reference);
    maxLegAngleDeg(index) = rad2deg(simulation.maxLegAngle);
    maxBodyPitchDeg(index) = rad2deg(simulation.maxBodyPitch);
    maxWheelTorque(index) = simulation.maxWheelTorque;
    stopTime(index) = reference.stopTime;
    withinSmallAngleRange(index) = simulation.isWithinSmallAngleRange;
end

sweepTable = table(accelerations, maxLegAngleDeg, maxBodyPitchDeg, ...
    maxWheelTorque, stopTime, withinSmallAngleRange, ...
    'VariableNames', {'Acceleration', 'MaxLegAngleDeg', ...
    'MaxBodyPitchDeg', 'MaxWheelTorque', 'StopTime', ...
    'Within15Deg'});

disp('=== 加速度扫描：同一 A/B/K，只改变速度斜率 ===');
disp(sweepTable);

validIndices = find(withinSmallAngleRange);
if isempty(validIndices)
    recommendedAcceleration = NaN;
    fprintf('扫描范围内没有满足暂定 15 deg 条件的加速度。\n');
else
    recommendedAcceleration = accelerations(validIndices(end));
    fprintf('扫描中满足暂定 15 deg 条件的最大加速度: %.1f m/s^2\n', ...
        recommendedAcceleration);
end
fprintf(['注意: 这是线性参考模型的筛选结果，不包含电机饱和、摩擦、' ...
    '离散控制和真实串联腿动力学。\n\n']);

if showPlots
    figureHandle = figure('Name', '六维模型加速度扫描', 'Color', 'w');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(accelerations, maxLegAngleDeg, 'o-', 'LineWidth', 1.3);
    yline(15, '--', '暂定小角度边界');
    grid on;
    ylabel('最大腿角 (deg)');
    title('速度指令越陡，平衡所需倾角越大');

    nexttile;
    plot(accelerations, maxWheelTorque, 'o-', 'LineWidth', 1.3);
    grid on;
    xlabel('最大加速度 (m/s^2)');
    ylabel('最大轮矩 (N*m)');
    result.figurePath = save_figure_6dof( ...
        figureHandle, "05_acceleration_sweep");
else
    result.figurePath = "";
end

result.table = sweepTable;
result.recommendedAcceleration = recommendedAcceleration;
result.smallAngleLimitDeg = 15;
end
