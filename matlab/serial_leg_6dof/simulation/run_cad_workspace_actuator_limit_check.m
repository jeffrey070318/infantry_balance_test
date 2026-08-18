function result = run_cad_workspace_actuator_limit_check(makeFigure)
%RUN_CAD_WORKSPACE_ACTUATOR_LIMIT_CHECK Scan VMC limits over output angles.

arguments
    makeFigure (1, 1) logical = true
end

parameters = cad_serial_leg_parameters();
actuators = serial_leg_actuator_parameters();
qReference = parameters.referenceOutputAngles;
offsetDegrees = linspace(-30, 30, 61);
gridSize = numel(offsetDegrees);

axialRatedLimitN = nan(gridSize);
legTorqueRatedLimitNm = nan(gridSize);
nominalRatedUtilization = nan(gridSize);
conditionNumber = nan(gridSize);
legLengthM = nan(gridSize);
nominalTaskForce = [120; 8];
jointRatedLimit = actuators.jointOutputRatedTorqueMagnitudeNm;

for row = 1:gridSize
    for column = 1:gridSize
        q = qReference + deg2rad([offsetDegrees(column); offsetDegrees(row)]);
        try
            [J, diagnostics] = cad_serial_leg_task_jacobian(q, parameters);
            axialRatedLimitN(row, column) = ...
                min(jointRatedLimit ./ abs(J(1, :).'));
            legTorqueRatedLimitNm(row, column) = ...
                min(jointRatedLimit ./ abs(J(2, :).'));
            nominalTorque = J.' * nominalTaskForce;
            nominalRatedUtilization(row, column) = ...
                max(abs(nominalTorque) ./ jointRatedLimit);
            conditionNumber(row, column) = diagnostics.conditionNumber;
            legLengthM(row, column) = diagnostics.pose.legLength;
        catch exception
            if ~strcmp(exception.identifier, 'sixdof:CadKinematics:Unreachable')
                rethrow(exception);
            end
        end
    end
end

reachable = isfinite(conditionNumber);
wellConditioned = reachable & conditionNumber <= 100;
if ~any(wellConditioned, 'all')
    error('sixdof:CadWorkspace:NoWellConditionedPose', ...
        '扫描范围内没有条件数不超过100的可用姿态。');
end

minimumAxialRatedLimitN = min(axialRatedLimitN(wellConditioned));
minimumLegTorqueRatedLimitNm = min(legTorqueRatedLimitNm(wellConditioned));
maximumNominalRatedUtilization = max(nominalRatedUtilization(wellConditioned));
maximumConditionNumber = max(conditionNumber(wellConditioned));
reachableFraction = mean(reachable, 'all');
wellConditionedFraction = mean(wellConditioned, 'all');
wellConditionedIndices = find(wellConditioned);
[~, minimumAxialLocalIndex] = ...
    min(axialRatedLimitN(wellConditionedIndices));
minimumAxialIndex = wellConditionedIndices(minimumAxialLocalIndex);
[minimumAxialRow, minimumAxialColumn] = ...
    ind2sub(size(axialRatedLimitN), minimumAxialIndex);
[~, maximumNominalLocalIndex] = ...
    max(nominalRatedUtilization(wellConditionedIndices));
maximumNominalIndex = wellConditionedIndices(maximumNominalLocalIndex);
[maximumNominalRow, maximumNominalColumn] = ...
    ind2sub(size(nominalRatedUtilization), maximumNominalIndex);
minimumAxialOffsetDegrees = ...
    [offsetDegrees(minimumAxialColumn); offsetDegrees(minimumAxialRow)];
maximumNominalOffsetDegrees = ...
    [offsetDegrees(maximumNominalColumn); offsetDegrees(maximumNominalRow)];

fprintf('±30 deg扫描几何可达比例: %.1f %%\n', 100 * reachableFraction);
fprintf('条件数<=100的可用比例: %.1f %%\n', 100 * wellConditionedFraction);
fprintf('可用区最小纯轴向额定力: %.2f N\n', minimumAxialRatedLimitN);
fprintf('可用区最小纯腿角额定力矩: %.2f N*m\n', ...
    minimumLegTorqueRatedLimitNm);
fprintf('[120 N; 8 N*m]最大额定利用率: %.3f\n', ...
    maximumNominalRatedUtilization);
fprintf('最小轴向承力姿态Δq1/Δq2: %.1f / %.1f deg\n', ...
    minimumAxialOffsetDegrees);
fprintf('典型指令最差姿态Δq1/Δq2: %.1f / %.1f deg\n', ...
    maximumNominalOffsetDegrees);

result.parameters = parameters;
result.actuators = actuators;
result.offsetDegrees = offsetDegrees;
result.reachable = reachable;
result.wellConditioned = wellConditioned;
result.axialRatedLimitN = axialRatedLimitN;
result.legTorqueRatedLimitNm = legTorqueRatedLimitNm;
result.nominalRatedUtilization = nominalRatedUtilization;
result.conditionNumber = conditionNumber;
result.legLengthM = legLengthM;
result.nominalTaskForce = nominalTaskForce;
result.minimumAxialRatedLimitN = minimumAxialRatedLimitN;
result.minimumLegTorqueRatedLimitNm = minimumLegTorqueRatedLimitNm;
result.maximumNominalRatedUtilization = maximumNominalRatedUtilization;
result.maximumConditionNumber = maximumConditionNumber;
result.reachableFraction = reachableFraction;
result.wellConditionedFraction = wellConditionedFraction;
result.minimumAxialOffsetDegrees = minimumAxialOffsetDegrees;
result.minimumAxialLegLengthM = ...
    legLengthM(minimumAxialRow, minimumAxialColumn);
result.maximumNominalOffsetDegrees = maximumNominalOffsetDegrees;
result.maximumNominalLegLengthM = ...
    legLengthM(maximumNominalRow, maximumNominalColumn);
result.figurePath = "";

if ~makeFigure
    return;
end

figureHandle = figure('Name', 'CAD workspace actuator limits', ...
    'Color', 'white', 'Position', [40, 60, 1200, 820]);
layout = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
drawMap(offsetDegrees, axialRatedLimitN, '纯轴向额定力上限 / N');
drawMap(offsetDegrees, legTorqueRatedLimitNm, ...
    '纯腿角额定力矩上限 / N·m');
drawMap(offsetDegrees, nominalRatedUtilization, ...
    '[120 N; 8 N·m]最大额定利用率');
drawMap(offsetDegrees, log10(conditionNumber), ...
    'log_{10}(cond(J))');
title(layout, {'真实CAD工作空间执行器边界', ...
    '横轴为q_1偏移，纵轴为q_2偏移；白区为闭环不可达'});

resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
if ~isfolder(resultDir), mkdir(resultDir); end
result.figurePath = fullfile(resultDir, ...
    '16_cad_workspace_actuator_limits.png');
exportgraphics(figureHandle, result.figurePath, 'Resolution', 180);
fprintf('工作空间执行器边界图已保存: %s\n', result.figurePath);
end

function drawMap(offsetDegrees, values, titleText)
nexttile;
imagesc(offsetDegrees, offsetDegrees, values);
set(gca, 'YDir', 'normal'); axis image; colorbar; grid on;
xlabel('Δq_1 / deg'); ylabel('Δq_2 / deg');
title(titleText);
end
