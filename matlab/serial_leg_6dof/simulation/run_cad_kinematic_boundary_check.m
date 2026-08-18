function result = run_cad_kinematic_boundary_check( ...
    makeFigure, offsetLimitDegrees, sampleCount)
%RUN_CAD_KINEMATIC_BOUNDARY_CHECK Find mathematical closure boundaries.

arguments
    makeFigure (1, 1) logical = true
    offsetLimitDegrees (1, 1) double {mustBePositive, mustBeFinite} = 90
    sampleCount (1, 1) double {mustBeInteger, mustBeGreaterThan(sampleCount, 2)} = 121
end

parameters = cad_serial_leg_parameters();
qReference = parameters.referenceOutputAngles;
offsetDegrees = linspace(-offsetLimitDegrees, offsetLimitDegrees, sampleCount);

legLengthM = nan(sampleCount);
legAngleDeg = nan(sampleCount);
conditionNumber = nan(sampleCount);
closureResidualM = nan(sampleCount);

for row = 1:sampleCount
    for column = 1:sampleCount
        q = qReference + deg2rad([offsetDegrees(column); offsetDegrees(row)]);
        try
            [~, diagnostics] = cad_serial_leg_task_jacobian(q, parameters);
            legLengthM(row, column) = diagnostics.pose.legLength;
            legAngleDeg(row, column) = rad2deg(diagnostics.pose.legAngle);
            conditionNumber(row, column) = diagnostics.conditionNumber;
            closureResidualM(row, column) = ...
                max(abs(diagnostics.pose.closureResidual));
        catch exception
            if ~strcmp(exception.identifier, 'sixdof:CadKinematics:Unreachable')
                rethrow(exception);
            end
        end
    end
end

reachable = isfinite(conditionNumber);
wellConditioned = reachable & conditionNumber <= 100;
nearSingular = reachable & ~wellConditioned;
if ~any(wellConditioned, 'all')
    error('sixdof:CadBoundary:NoWellConditionedPose', ...
        '扩展扫描范围内没有条件数不超过100的姿态。');
end

minimumLegLengthM = min(legLengthM(wellConditioned));
maximumLegLengthM = max(legLengthM(wellConditioned));
minimumLegAngleDeg = min(legAngleDeg(wellConditioned));
maximumLegAngleDeg = max(legAngleDeg(wellConditioned));
maximumClosureResidualM = max(closureResidualM(wellConditioned));
reachableFraction = mean(reachable, 'all');
nearSingularFraction = mean(nearSingular, 'all');

fprintf('±%.0f deg扩展扫描几何可达比例: %.1f %%\n', ...
    offsetLimitDegrees, 100 * reachableFraction);
fprintf('可达区中cond(J)>100比例: %.1f %%\n', ...
    100 * nearSingularFraction / reachableFraction);
fprintf('条件数<=100的数学腿长范围: %.4f ~ %.4f m\n', ...
    minimumLegLengthM, maximumLegLengthM);
fprintf('条件数<=100的数学腿角范围: %.2f ~ %.2f deg\n', ...
    minimumLegAngleDeg, maximumLegAngleDeg);
fprintf('可用区最大闭环残差: %.3e m\n', maximumClosureResidualM);

result.parameters = parameters;
result.offsetLimitDegrees = offsetLimitDegrees;
result.sampleCount = sampleCount;
result.offsetDegrees = offsetDegrees;
result.reachable = reachable;
result.wellConditioned = wellConditioned;
result.nearSingular = nearSingular;
result.legLengthM = legLengthM;
result.legAngleDeg = legAngleDeg;
result.conditionNumber = conditionNumber;
result.closureResidualM = closureResidualM;
result.reachableFraction = reachableFraction;
result.nearSingularFraction = nearSingularFraction;
result.minimumLegLengthM = minimumLegLengthM;
result.maximumLegLengthM = maximumLegLengthM;
result.minimumLegAngleDeg = minimumLegAngleDeg;
result.maximumLegAngleDeg = maximumLegAngleDeg;
result.maximumClosureResidualM = maximumClosureResidualM;
result.figurePath = "";

if ~makeFigure
    return;
end

figureHandle = figure('Name', 'CAD kinematic boundary', ...
    'Color', 'white', 'Position', [40, 60, 1200, 820]);
layout = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

classification = double(reachable) + double(wellConditioned);
nexttile;
imagesc(offsetDegrees, offsetDegrees, classification);
set(gca, 'YDir', 'normal'); axis image; grid on;
colormap(gca, [0.82 0.82 0.82; 0.95 0.55 0.20; 0.20 0.65 0.35]);
colorbar('Ticks', [0, 1, 2], ...
    'TickLabels', {'不可达', 'cond(J)>100', '条件正常'});
xlabel('Δq_1 / deg'); ylabel('Δq_2 / deg');
title('闭环可达与条件数分类');

drawMap(offsetDegrees, 1000 * legLengthM, '等效腿长 / mm');
drawMap(offsetDegrees, legAngleDeg, '等效腿角 / deg');
drawMap(offsetDegrees, log10(conditionNumber), 'log_{10}(cond(J))');
title(layout, {'真实CAD闭环的数学运动边界', ...
    '不包含实体碰撞、机械止挡、线束和链条干涉'});

resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
if ~isfolder(resultDir), mkdir(resultDir); end
result.figurePath = fullfile(resultDir, '17_cad_kinematic_boundary.png');
exportgraphics(figureHandle, result.figurePath, 'Resolution', 180);
fprintf('CAD数学运动边界图已保存: %s\n', result.figurePath);
end

function drawMap(offsetDegrees, values, titleText)
nexttile;
imagesc(offsetDegrees, offsetDegrees, values);
set(gca, 'YDir', 'normal'); axis image; colorbar; grid on;
xlabel('Δq_1 / deg'); ylabel('Δq_2 / deg');
title(titleText);
end
