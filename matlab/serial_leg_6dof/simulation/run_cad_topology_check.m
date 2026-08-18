function result = run_cad_topology_check(makeFigure)
%RUN_CAD_TOPOLOGY_CHECK Inspect the joint graph extracted from the team CAD.

arguments
    makeFigure (1, 1) logical = true
end

geometry = load_cad_serial_leg_geometry();
directionError = max(vecnorm(geometry.axisDirections - [1; 0; 0], 2, 1));
planeSpreadMm = range(geometry.pointsMm(1, :));

fprintf('CAD候选关节轴: %d 条\n', size(geometry.pointsMm, 2));
fprintf('运动平面法向与装配X轴最大偏差: %.3e\n', directionError);
fprintf('候选轴投影平面离散量: %.3e mm\n', planeSpreadMm);
fprintf('注意: STEP不包含链轮/齿轮约束，当前结果还不是二自由度正运动学。\n');

result.geometry = geometry;
result.directionError = directionError;
result.planeSpreadMm = planeSpreadMm;
result.figurePath = "";

if ~makeFigure
    return;
end

figureHandle = figure('Name', 'Team CAD joint topology', ...
    'Color', 'white', 'Position', [80, 80, 1100, 700]);
hold on;
componentNames = string({geometry.linkAxisDistances.component});
uniqueComponents = unique(componentNames, 'stable');
colors = lines(numel(uniqueComponents));
for index = 1:numel(geometry.linkAxisDistances)
    link = geometry.linkAxisDistances(index);
    fromIndex = link.from_candidate;
    toIndex = link.to_candidate;
    points = geometry.planePointsM(:, [fromIndex, toIndex]);
    colorIndex = find(uniqueComponents == string(link.component), 1);
    plot(points(1, :), points(2, :), 'o-', 'LineWidth', 2.2, ...
        'Color', colors(colorIndex, :), 'MarkerFaceColor', 'white', ...
        'HandleVisibility', 'off');
    midpoint = mean(points, 2);
    segment = points(:, 2) - points(:, 1);
    normal = [-segment(2); segment(1)] / norm(segment);
    labelPoint = midpoint + (-1)^index * 0.004 * normal;
    text(labelPoint(1), labelPoint(2), ...
        sprintf('%.2f mm', link.distance_mm), ...
        'FontSize', 9, 'HorizontalAlignment', 'center', ...
        'BackgroundColor', 'white', 'Margin', 1);
end

legendHandles = gobjects(numel(uniqueComponents), 1);
for index = 1:numel(uniqueComponents)
    legendHandles(index) = plot(nan, nan, 'o-', 'LineWidth', 2.2, ...
        'Color', colors(index, :), 'MarkerFaceColor', 'white');
end

for index = 1:size(geometry.planePointsM, 2)
    point = geometry.planePointsM(:, index);
    text(point(1), point(2), sprintf('  J%d', index), ...
        'FontWeight', 'bold', 'VerticalAlignment', 'bottom');
end
axis equal; grid on;
xlabel('SolidWorks装配Y / m');
ylabel('SolidWorks装配Z / m');
title({'本队串联腿STEP候选关节拓扑', ...
    '轴距来自CAD；链轮/齿轮约束、主动轴与编码器零位仍待确认'});
legend(legendHandles, uniqueComponents, 'Location', 'southoutside', ...
    'Orientation', 'horizontal', 'NumColumns', 3);

resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
if ~isfolder(resultDir), mkdir(resultDir); end
result.figurePath = fullfile(resultDir, '12_cad_joint_topology.png');
exportgraphics(figureHandle, result.figurePath, 'Resolution', 180);
fprintf('CAD拓扑图已保存: %s\n', result.figurePath);
end
