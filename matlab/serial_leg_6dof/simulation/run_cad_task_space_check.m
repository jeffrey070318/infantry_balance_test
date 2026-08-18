function result = run_cad_task_space_check(makeFigure)
%RUN_CAD_TASK_SPACE_CHECK Validate wheel-based task coordinates and VMC.

arguments
    makeFigure (1, 1) logical = true
end

parameters = cad_serial_leg_parameters();
qReference = parameters.referenceOutputAngles;
[jacobian, diagnostics] = cad_serial_leg_task_jacobian(qReference, parameters);
wheelError = norm(diagnostics.pose.wheelCenter - ...
    parameters.wheelCenterReference);
taskForce = [120; 8];
qDot = [0.7; -0.4];
vmc = cad_task_force_to_output_torque(taskForce, qReference, parameters);
taskVelocity = jacobian * qDot;
powerResidual = vmc.outputTorque.' * qDot - taskForce.' * taskVelocity;

fprintf('轮心刚性偏置回代误差: %.3e m\n', wheelError);
fprintf('参考等效腿长: %.6f m\n', diagnostics.pose.legLength);
fprintf('参考等效腿角: %.6f deg\n', rad2deg(diagnostics.pose.legAngle));
fprintf('任务Jacobian条件数: %.3f\n', diagnostics.conditionNumber);
fprintf('真实CAD任务空间虚功残差: %.3e W\n', abs(powerResidual));

result.parameters = parameters;
result.jacobian = jacobian;
result.diagnostics = diagnostics;
result.vmc = vmc;
result.wheelError = wheelError;
result.powerResidual = powerResidual;
result.figurePath = "";

if ~makeFigure
    return;
end

offsetDegrees = linspace(-15, 15, 51);
legLengthMm = nan(numel(offsetDegrees));
legAngleDeg = nan(numel(offsetDegrees));
conditionNumber = nan(numel(offsetDegrees));
for row = 1:numel(offsetDegrees)
    for column = 1:numel(offsetDegrees)
        q = qReference + deg2rad([offsetDegrees(column); offsetDegrees(row)]);
        try
            [localJacobian, localDiagnostics] = ...
                cad_serial_leg_task_jacobian(q, parameters);
            legLengthMm(row, column) = 1000 * localDiagnostics.pose.legLength;
            legAngleDeg(row, column) = rad2deg(localDiagnostics.pose.legAngle);
            conditionNumber(row, column) = cond(localJacobian);
        catch exception
            if ~strcmp(exception.identifier, 'sixdof:CadKinematics:Unreachable')
                rethrow(exception);
            end
        end
    end
end

figureHandle = figure('Name', 'CAD task-space map', ...
    'Color', 'white', 'Position', [60, 100, 1200, 430]);
layout = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
drawMap(offsetDegrees, legLengthMm, '等效腿长 / mm');
drawMap(offsetDegrees, legAngleDeg, '等效腿角 / deg');
drawMap(offsetDegrees, log10(conditionNumber), 'log_{10}(cond(J))');
title(layout, {'本队串联腿CAD任务空间', ...
    '横轴为q_1偏移，纵轴为q_2偏移；白区表示闭环不可达'});

resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
if ~isfolder(resultDir), mkdir(resultDir); end
result.figurePath = fullfile(resultDir, '14_cad_task_space_map.png');
exportgraphics(figureHandle, result.figurePath, 'Resolution', 180);
fprintf('CAD任务空间图已保存: %s\n', result.figurePath);
end

function drawMap(offsetDegrees, values, titleText)
nexttile;
imagesc(offsetDegrees, offsetDegrees, values);
set(gca, 'YDir', 'normal'); axis image;
colorbar; grid on;
xlabel('Δq_1 / deg'); ylabel('Δq_2 / deg');
title(titleText);
end
