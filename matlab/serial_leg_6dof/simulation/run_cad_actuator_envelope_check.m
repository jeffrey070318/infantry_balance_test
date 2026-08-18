function result = run_cad_actuator_envelope_check(makeFigure)
%RUN_CAD_ACTUATOR_ENVELOPE_CHECK Map CAD VMC commands to DM8009 limits.

arguments
    makeFigure (1, 1) logical = true
end

parameters = cad_serial_leg_parameters();
actuators = serial_leg_actuator_parameters();
q = parameters.referenceOutputAngles;
[J, diagnostics] = cad_serial_leg_task_jacobian(q, parameters);

axialForceN = linspace(-500, 500, 161);
legTorqueNm = linspace(-50, 50, 161);
maxRatedUtilization = zeros(numel(legTorqueNm), numel(axialForceN));
for row = 1:numel(legTorqueNm)
    taskForces = [axialForceN; repmat(legTorqueNm(row), 1, numel(axialForceN))];
    outputTorques = J.' * taskForces;
    utilizations = abs(outputTorques) ./ ...
        actuators.jointOutputRatedTorqueMagnitudeNm;
    maxRatedUtilization(row, :) = max(utilizations, [], 1);
end

ratedMask = maxRatedUtilization <= 1;
peakMask = maxRatedUtilization <= 2;
ratedAreaFraction = mean(ratedMask, 'all');
peakAreaFraction = mean(peakMask, 'all');

fprintf('参考姿态任务Jacobian条件数: %.3f\n', diagnostics.conditionNumber);
fprintf('扫描窗口内DM8009额定可行比例: %.1f %%\n', 100 * ratedAreaFraction);
fprintf('扫描窗口内DM8009峰值可行比例: %.1f %%\n', 100 * peakAreaFraction);
fprintf('轮端理想连续/峰值转矩: %.2f / %.2f N*m\n', ...
    actuators.wheelTransmission.continuousWheelTorqueIdealNmCandidate, ...
    actuators.wheelTransmission.peakWheelTorqueIdealNmCandidate);

result.parameters = parameters;
result.actuators = actuators;
result.jacobian = J;
result.axialForceN = axialForceN;
result.legTorqueNm = legTorqueNm;
result.maxRatedUtilization = maxRatedUtilization;
result.ratedAreaFraction = ratedAreaFraction;
result.peakAreaFraction = peakAreaFraction;
result.figurePath = "";

if ~makeFigure
    return;
end

figureHandle = figure('Name', 'CAD VMC actuator envelope', ...
    'Color', 'white', 'Position', [80, 100, 1050, 430]);
layout = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
imagesc(axialForceN, legTorqueNm, maxRatedUtilization);
set(gca, 'YDir', 'normal'); axis tight; colorbar; grid on;
hold on;
contour(axialForceN, legTorqueNm, maxRatedUtilization, [1, 1], ...
    'k', 'LineWidth', 1.8);
contour(axialForceN, legTorqueNm, maxRatedUtilization, [2, 2], ...
    'r', 'LineWidth', 1.8);
xlabel('轴向力 F_0 / N'); ylabel('等效腿角力矩 T_p / N·m');
title('最大关节额定转矩利用率');

nexttile;
classification = double(ratedMask) + double(peakMask);
imagesc(axialForceN, legTorqueNm, classification);
set(gca, 'YDir', 'normal'); axis tight; grid on;
colormap(gca, [0.75 0.20 0.20; 0.95 0.70 0.20; 0.20 0.65 0.35]);
colorbar('Ticks', [0, 1, 2], ...
    'TickLabels', {'超过峰值', '仅峰值可行', '额定可行'});
xlabel('轴向力 F_0 / N'); ylabel('等效腿角力矩 T_p / N·m');
title('DM8009双关节可行域');
title(layout, {'真实CAD参考姿态的VMC执行器边界', ...
    '黑线=20 N·m额定边界，红线=40 N·m峰值边界'});

resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
if ~isfolder(resultDir), mkdir(resultDir); end
result.figurePath = fullfile(resultDir, '15_cad_vmc_actuator_envelope.png');
exportgraphics(figureHandle, result.figurePath, 'Resolution', 180);
fprintf('VMC执行器可行域图已保存: %s\n', result.figurePath);
end
