function result = run_fixed_leg_controller_robustness_6dof(makeFigure)
%RUN_FIXED_LEG_CONTROLLER_ROBUSTNESS_6DOF Compare sampled saturated K choices.

arguments
    makeFigure (1, 1) logical = true
end

totalMass = 19;
legLength = team_firmware_parameters().legLengthDefault;
parameters = team_estimated_dynamics_parameters(totalMass, legLength);
model = linearize_equivalent_leg_6dof(parameters);
controllers(1).name = "同构固件K";
controllers(1).data.K = firmware_gain_schedule_6dof(legLength).K;
controllers(2).name = "19 kg重算K";
controllers(2).data = design_heu_lqr_6dof(model);

cad = cad_serial_leg_parameters();
inverse = cad_serial_leg_inverse_task_pose( ...
    [legLength; 0], cad.referenceOutputAngles, cad);
outputAngles = inverse.outputAngles;
pose = inverse.pose;
axialForceN = totalMass * parameters.gravity / (2 * cos(pose.legAngle));
sampleTimes = [0.001; 0.002; 0.005; 0.010; 0.015; 0.020; 0.025; 0.030];
disturbanceDegrees = [2; 5; 10; 15; 20; 30; 40];
simulationDuration = 5;
simulation = cell(numel(sampleTimes), numel(disturbanceDegrees), numel(controllers));
converged = false(size(simulation));
diverged = false(size(simulation));
jointSaturationFraction = zeros(size(simulation));
wheelSaturationFraction = zeros(size(simulation));
spectralRadius = zeros(numel(sampleTimes), numel(controllers));

for controllerIndex = 1:numel(controllers)
    for sampleIndex = 1:numel(sampleTimes)
        for disturbanceIndex = 1:numel(disturbanceDegrees)
            current = simulate_sampled_saturated_disturbance_6dof( ...
                model, controllers(controllerIndex).data, ...
                deg2rad(disturbanceDegrees(disturbanceIndex)), ...
                sampleTimes(sampleIndex), simulationDuration, ...
                outputAngles, axialForceN);
            simulation{sampleIndex, disturbanceIndex, controllerIndex} = current;
            converged(sampleIndex, disturbanceIndex, controllerIndex) = current.isConverged;
            diverged(sampleIndex, disturbanceIndex, controllerIndex) = current.isDiverged;
            jointSaturationFraction(sampleIndex, disturbanceIndex, controllerIndex) = ...
                current.jointSaturationFraction;
            wheelSaturationFraction(sampleIndex, disturbanceIndex, controllerIndex) = ...
                current.wheelSaturationFraction;
            spectralRadius(sampleIndex, controllerIndex) = current.spectralRadius;
        end
    end
end

maximumConvergedDisturbanceDeg = zeros(numel(sampleTimes), numel(controllers));
for controllerIndex = 1:numel(controllers)
    for sampleIndex = 1:numel(sampleTimes)
        valid = disturbanceDegrees(converged(sampleIndex, :, controllerIndex));
        if isempty(valid)
            maximumConvergedDisturbanceDeg(sampleIndex, controllerIndex) = 0;
        else
            maximumConvergedDisturbanceDeg(sampleIndex, controllerIndex) = max(valid);
        end
    end
end

fprintf('固定腿长离散/饱和/大扰动检查: 19 kg, L0=%.3f m\n', legLength);
fprintf('CAD求解姿态: q=[%.3f, %.3f] deg, 腿角=%.6f deg\n', ...
    rad2deg(outputAngles), rad2deg(pose.legAngle));
fprintf('单腿轴向支撑前馈: %.3f N\n', axialForceN);
for controllerIndex = 1:numel(controllers)
    fprintf('  %s: 1 ms最大收敛扰动 %.0f deg, 线性离散稳定上限扫描点 %.0f ms\n', ...
        controllers(controllerIndex).name, ...
        maximumConvergedDisturbanceDeg(1, controllerIndex), ...
        1000 * max(sampleTimes(spectralRadius(:, controllerIndex) < 1)));
end

result.totalMass = totalMass;
result.legLength = legLength;
result.model = model;
result.controllers = controllers;
result.outputAngles = outputAngles;
result.pose = pose;
result.axialForceN = axialForceN;
result.sampleTimes = sampleTimes;
result.disturbanceDegrees = disturbanceDegrees;
result.simulation = simulation;
result.converged = converged;
result.diverged = diverged;
result.jointSaturationFraction = jointSaturationFraction;
result.wheelSaturationFraction = wheelSaturationFraction;
result.spectralRadius = spectralRadius;
result.maximumConvergedDisturbanceDeg = maximumConvergedDisturbanceDeg;
result.figurePath = "";
result.warning = [
    "结果不含传感器噪声、计算延迟、丢包、摩擦和轮地打滑"
    "大扰动收敛只代表当前暂估非线性方程，不是实机安全倾角"
    ];

if makeFigure
    figureHandle = figure('Name', 'Fixed leg controller robustness', ...
        'Color', 'white', 'Position', [100, 100, 1240, 800]);
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    for controllerIndex = 1:numel(controllers)
        nexttile;
        imagesc(disturbanceDegrees, 1000 * sampleTimes, ...
            converged(:, :, controllerIndex));
        axis xy; colormap(gca, [0.85 0.25 0.20; 0.20 0.65 0.35]);
        colorbar('Ticks', [0.25, 0.75], 'TickLabels', {'未收敛', '收敛'});
        xlabel('初始机体俯仰扰动 / deg'); ylabel('控制周期 / ms');
        title(controllers(controllerIndex).name + "：5 s收敛区域");
    end

    nexttile;
    selectedSample = 1;
    selectedDisturbance = find(disturbanceDegrees == 10, 1);
    hold on;
    for controllerIndex = 1:numel(controllers)
        current = simulation{selectedSample, selectedDisturbance, controllerIndex};
        plot(current.time, rad2deg(current.state(:, 5)), 'LineWidth', 1.5, ...
            'DisplayName', controllers(controllerIndex).name);
    end
    grid on; xlabel('时间 / s'); ylabel('机体俯仰角 / deg');
    title('1 ms、10 deg扰动响应'); legend('Location', 'best');

    nexttile;
    hold on;
    for controllerIndex = 1:numel(controllers)
        plot(disturbanceDegrees, 100 * squeeze( ...
            jointSaturationFraction(1, :, controllerIndex)), 'o-', ...
            'LineWidth', 1.4, 'DisplayName', controllers(controllerIndex).name);
    end
    grid on; xlabel('初始机体俯仰扰动 / deg');
    ylabel('DM8009额定饱和采样占比 / %');
    title('1 ms控制下的关节饱和'); legend('Location', 'best');

    resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
    if ~isfolder(resultDir), mkdir(resultDir); end
    result.figurePath = fullfile(resultDir, '21_fixed_leg_controller_robustness.png');
    exportgraphics(figureHandle, result.figurePath, 'Resolution', 160);
    fprintf('固定腿长鲁棒性图已保存: %s\n', result.figurePath);
end
