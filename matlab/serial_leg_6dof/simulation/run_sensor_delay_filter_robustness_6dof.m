function result = run_sensor_delay_filter_robustness_6dof(makeFigure, trialCount)
%RUN_SENSOR_DELAY_FILTER_ROBUSTNESS_6DOF Monte Carlo digital-control check.

arguments
    makeFigure (1, 1) logical = true
    trialCount (1, 1) double {mustBeInteger, mustBePositive} = 8
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
sampleTime = 0.001;
duration = 4;
initialPitchRad = deg2rad(10);
nominalNoiseStd = [deg2rad(0.08); deg2rad(0.5); 0.002; 0.02; ...
    deg2rad(0.05); deg2rad(0.3)];

scenarios(1) = makeScenario("理想", zeros(6, 1), ones(6, 1), 0);
scenarios(2) = makeScenario("仅噪声", nominalNoiseStd, ones(6, 1), 0);
scenarios(3) = makeScenario("固件式滤波+1拍", nominalNoiseStd, ...
    [1; 1; 1; 0.5; 1; 1], 1);
scenarios(4) = makeScenario("全状态alpha=0.5+1拍", nominalNoiseStd, ...
    0.5 * ones(6, 1), 1);

shape = [numel(scenarios), numel(controllers), trialCount];
converged = false(shape);
pitchRmsDeg = nan(shape);
wheelTorqueRmsNm = nan(shape);
jointTorqueStdNm = nan([shape, 2]);
representative = cell(numel(scenarios), numel(controllers));

for scenarioIndex = 1:numel(scenarios)
    for controllerIndex = 1:numel(controllers)
        for trialIndex = 1:trialCount
            configuration = scenarios(scenarioIndex).configuration;
            configuration.seed = 1000 * scenarioIndex + trialIndex;
            current = simulate_noisy_delayed_disturbance_6dof( ...
                model, controllers(controllerIndex).data, initialPitchRad, ...
                sampleTime, duration, outputAngles, axialForceN, configuration, cad);
            converged(scenarioIndex, controllerIndex, trialIndex) = current.isConverged;
            pitchRmsDeg(scenarioIndex, controllerIndex, trialIndex) = ...
                current.steadyPitchRmsDeg;
            wheelTorqueRmsNm(scenarioIndex, controllerIndex, trialIndex) = ...
                current.steadyWheelTorqueRmsNm;
            jointTorqueStdNm(scenarioIndex, controllerIndex, trialIndex, :) = ...
                current.steadyJointTorqueStdNm;
            if trialIndex == 1
                representative{scenarioIndex, controllerIndex} = current;
            end
        end
    end
end

convergenceRate = mean(converged, 3);
meanPitchRmsDeg = mean(pitchRmsDeg, 3);
meanWheelTorqueRmsNm = mean(wheelTorqueRmsNm, 3);
meanJointTorqueStdNm = mean(jointTorqueStdNm, 3);

fprintf('传感器噪声/滤波/一拍延迟检查: %d次固定种子重复\n', trialCount);
for scenarioIndex = 1:numel(scenarios)
    fprintf('  %s\n', scenarios(scenarioIndex).name);
    for controllerIndex = 1:numel(controllers)
        fprintf('    %s: 收敛率 %.1f%%, 稳态pitch RMS %.4f deg, 轮矩RMS %.4f N*m\n', ...
            controllers(controllerIndex).name, ...
            100 * convergenceRate(scenarioIndex, controllerIndex), ...
            meanPitchRmsDeg(scenarioIndex, controllerIndex), ...
            meanWheelTorqueRmsNm(scenarioIndex, controllerIndex));
    end
end

result.totalMass = totalMass;
result.legLength = legLength;
result.model = model;
result.controllers = controllers;
result.scenarios = scenarios;
result.trialCount = trialCount;
result.outputAngles = outputAngles;
result.pose = pose;
result.axialForceN = axialForceN;
result.converged = converged;
result.pitchRmsDeg = pitchRmsDeg;
result.wheelTorqueRmsNm = wheelTorqueRmsNm;
result.jointTorqueStdNm = jointTorqueStdNm;
result.convergenceRate = convergenceRate;
result.meanPitchRmsDeg = meanPitchRmsDeg;
result.meanWheelTorqueRmsNm = meanWheelTorqueRmsNm;
result.meanJointTorqueStdNm = meanJointTorqueStdNm;
result.representative = representative;
result.figurePath = "";
result.warning = [
    "噪声幅值是工程假设，必须在实机静止日志后更新"
    "当前只模拟白噪声，没有零偏漂移、时间戳抖动和丢包"
    ];

if makeFigure
    scenarioNames = string({scenarios.name});
    figureHandle = figure('Name', 'Sensor delay filter robustness', ...
        'Color', 'white', 'Position', [100, 100, 1280, 800]);
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    bar(100 * convergenceRate);
    ylim([0, 105]); grid on; ylabel('收敛率 / %');
    set(gca, 'XTickLabel', scenarioNames, 'XTickLabelRotation', 15);
    title(sprintf('%d次固定种子重复', trialCount));
    legend(string({controllers.name}), 'Location', 'best');

    nexttile;
    bar(meanPitchRmsDeg);
    grid on; ylabel('稳态机体俯仰 RMS / deg');
    set(gca, 'XTickLabel', scenarioNames, 'XTickLabelRotation', 15);
    title('最后1 s姿态抖动'); legend(string({controllers.name}), 'Location', 'best');

    nexttile;
    bar(meanWheelTorqueRmsNm);
    grid on; ylabel('稳态轮矩 RMS / N m');
    set(gca, 'XTickLabel', scenarioNames, 'XTickLabelRotation', 15);
    title('噪声引起的控制输出'); legend(string({controllers.name}), 'Location', 'best');

    nexttile;
    scenarioIndex = 3;
    hold on;
    for controllerIndex = 1:numel(controllers)
        current = representative{scenarioIndex, controllerIndex};
        plot(current.time, rad2deg(current.state(:, 5)), 'LineWidth', 1.4, ...
            'DisplayName', controllers(controllerIndex).name);
    end
    grid on; xlabel('时间 / s'); ylabel('机体俯仰角 / deg');
    title('固件式滤波+1拍延迟，10 deg扰动'); legend('Location', 'best');

    resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
    if ~isfolder(resultDir), mkdir(resultDir); end
    result.figurePath = fullfile(resultDir, '22_sensor_delay_filter_robustness.png');
    exportgraphics(figureHandle, result.figurePath, 'Resolution', 160);
    fprintf('噪声/延迟/滤波图已保存: %s\n', result.figurePath);
end
end

function scenario = makeScenario(name, noiseStd, filterAlpha, delaySamples)
scenario.name = name;
scenario.configuration.noiseStd = noiseStd;
scenario.configuration.filterAlpha = filterAlpha;
scenario.configuration.delaySamples = delaySamples;
scenario.configuration.seed = 0;
end
