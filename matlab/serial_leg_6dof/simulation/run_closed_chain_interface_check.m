function result = run_closed_chain_interface_check(makeFigure)
%RUN_CLOSED_CHAIN_INTERFACE_CHECK Verify the placeholder mechanism layer.

arguments
    makeFigure (1, 1) logical = true
end

geometry = placeholder_serial_leg_geometry();
jointAngles = deg2rad([40; 140]);
[jacobian, diagnostics] = closed_chain_task_jacobian(jointAngles, geometry);
step = 1e-7;
finiteDifferenceJacobian = zeros(2);
for column = 1:2
    delta = zeros(2, 1); delta(column) = step;
    plus = closed_chain_forward_kinematics(jointAngles + delta, geometry);
    minus = closed_chain_forward_kinematics(jointAngles - delta, geometry);
    finiteDifferenceJacobian(:, column) = ...
        ([plus.legLength; plus.legAngle] - ...
        [minus.legLength; minus.legAngle]) / (2 * step);
end

taskForce = [120; 8];
vmc = task_force_to_joint_torque(taskForce, jointAngles, geometry);
jointVelocity = [0.7; -0.4];
taskVelocity = jacobian * jointVelocity;
powerResidual = vmc.jointTorque.' * jointVelocity - ...
    taskForce.' * taskVelocity;

fprintf('闭环长度残差: %.3e m\n', max(abs(diagnostics.pose.closureResidual)));
fprintf('Jacobian有限差分误差: %.3e\n', ...
    max(abs(jacobian - finiteDifferenceJacobian), [], 'all'));
fprintf('虚功残差: %.3e W\n', abs(powerResidual));
fprintf('注意: 当前杆长是占位参数，不代表交龙或本队实机。\n');

result.geometry = geometry;
result.jointAngles = jointAngles;
result.jacobian = jacobian;
result.finiteDifferenceJacobian = finiteDifferenceJacobian;
result.diagnostics = diagnostics;
result.vmc = vmc;
result.powerResidual = powerResidual;
result.figurePath = "";

if makeFigure
    pose = diagnostics.pose;
    figureHandle = figure('Name', 'Closed-chain interface check', ...
        'Color', 'white', 'Position', [100, 100, 760, 620]);
    plot([geometry.origin(1), pose.pointA(1), pose.endpoint(1)], ...
        [geometry.origin(2), pose.pointA(2), pose.endpoint(2)], ...
        'o-', 'LineWidth', 2);
    hold on;
    plot([geometry.origin(1), pose.pointB(1), pose.endpoint(1)], ...
        [geometry.origin(2), pose.pointB(2), pose.endpoint(2)], ...
        'o-', 'LineWidth', 2);
    plot([geometry.origin(1), pose.endpoint(1)], ...
        [geometry.origin(2), pose.endpoint(2)], 'k--', 'LineWidth', 1.2);
    axis equal; grid on;
    xlabel('车体坐标 x / m'); ylabel('车体坐标 z / m');
    title('闭环机构接口检查（占位几何，不是实机尺寸）');
    legend('主动杆1-被动杆1', '主动杆2-被动杆2', '等效腿');

    resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
    if ~isfolder(resultDir), mkdir(resultDir); end
    result.figurePath = fullfile(resultDir, '11_closed_chain_interface_check.png');
    exportgraphics(figureHandle, result.figurePath, 'Resolution', 160);
    fprintf('机构检查图已保存: %s\n', result.figurePath);
end
end
