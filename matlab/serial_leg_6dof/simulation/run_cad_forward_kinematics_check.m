function result = run_cad_forward_kinematics_check(makeFigure)
%RUN_CAD_FORWARD_KINEMATICS_CHECK Validate two-output CAD loop reconstruction.

arguments
    makeFigure (1, 1) logical = true
end

parameters = cad_serial_leg_parameters();
referencePose = cad_serial_leg_forward_kinematics( ...
    parameters.referenceOutputAngles, parameters);
referenceError = max(abs(referencePose.nodes - parameters.referenceNodes), [], 'all');
closureError = max(abs(referencePose.closureResidual));
distalJacobian = cad_serial_leg_node_jacobian( ...
    parameters.referenceOutputAngles, 7, parameters);

fprintf('CAD二输出正解回代误差: %.3e m\n', referenceError);
fprintf('CAD闭环最大长度/向量残差: %.3e m\n', closureError);
fprintf('J7参考姿态Jacobian条件数: %.3f\n', cond(distalJacobian));
fprintf('注意: q是J1输出角，不是电机编码器角；J7尚未认定为轮轴。\n');

result.parameters = parameters;
result.referencePose = referencePose;
result.referenceError = referenceError;
result.closureError = closureError;
result.distalJacobian = distalJacobian;
result.figurePath = "";

if ~makeFigure
    return;
end

offsetsDeg = [-8, 0; 0, 0; 0, 8; 8, 0; 0, -8];
colors = lines(size(offsetsDeg, 1));
figureHandle = figure('Name', 'CAD two-output kinematics', ...
    'Color', 'white', 'Position', [60, 60, 1200, 760]);
layout = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
poses = cell(size(offsetsDeg, 1), 1);
allPoints = [];
for poseIndex = 1:size(offsetsDeg, 1)
    q = parameters.referenceOutputAngles + deg2rad(offsetsDeg(poseIndex, :).');
    poses{poseIndex} = cad_serial_leg_forward_kinematics(q, parameters);
    allPoints = [allPoints, poses{poseIndex}.nodes]; %#ok<AGROW>
end
margin = 0.015;
xLimits = [min(allPoints(1, :)) - margin, max(allPoints(1, :)) + margin];
yLimits = [min(allPoints(2, :)) - margin, max(allPoints(2, :)) + margin];
for poseIndex = 1:numel(poses)
    nexttile;
    hold on;
    drawPose(poses{poseIndex}.nodes, colors(poseIndex, :), true);
    axis equal; grid on;
    xlim(xLimits); ylim(yLimits);
    xlabel('装配Y / m'); ylabel('装配Z / m');
    title(sprintf('Δq_1=%+d°,  Δq_2=%+d°', ...
        offsetsDeg(poseIndex, 1), offsetsDeg(poseIndex, 2)));
end
title(layout, {'本队串联腿CAD二输出正运动学', ...
    '彩色姿态为输出角分别扰动±8°；J7仅为远端机构参考点'});

resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
if ~isfolder(resultDir), mkdir(resultDir); end
result.figurePath = fullfile(resultDir, '13_cad_two_output_kinematics.png');
exportgraphics(figureHandle, result.figurePath, 'Resolution', 180);
fprintf('CAD二输出运动学图已保存: %s\n', result.figurePath);
end

function drawPose(nodes, color, labelNodes)
segments = [1, 2; 2, 3; 1, 4; 4, 3; 4, 5; 3, 5; ...
    1, 6; 4, 6; 5, 7; 6, 7];
for segment = segments.'
    plot(nodes(1, segment), nodes(2, segment), 'o-', ...
        'Color', color, 'LineWidth', 1.8, 'MarkerSize', 4, ...
        'HandleVisibility', 'off');
end
if labelNodes
    for node = 1:7
        text(nodes(1, node), nodes(2, node), sprintf(' J%d', node), ...
            'FontWeight', 'bold', 'FontSize', 9);
    end
end
end
