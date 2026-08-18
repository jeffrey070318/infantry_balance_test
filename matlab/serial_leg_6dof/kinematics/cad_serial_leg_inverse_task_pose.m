function result = cad_serial_leg_inverse_task_pose( ...
    targetTask, initialOutputAngles, parameters)
%CAD_SERIAL_LEG_INVERSE_TASK_POSE Solve [L0; legAngle] by Newton iteration.

arguments
    targetTask (2, 1) double {mustBeFinite}
    initialOutputAngles (2, 1) double {mustBeFinite}
    parameters (1, 1) struct = cad_serial_leg_parameters()
end

outputAngles = initialOutputAngles;
maximumIterations = 15;
for iteration = 1:maximumIterations
    pose = cad_serial_leg_forward_kinematics(outputAngles, parameters);
    residual = targetTask - [pose.legLength; pose.legAngle];
    if norm(residual, inf) < 1e-11
        result = packageResult(outputAngles, pose, residual, iteration);
        return;
    end
    jacobian = cad_serial_leg_task_jacobian(outputAngles, parameters);
    if rcond(jacobian) < 1e-8
        error('sixdof:CadKinematics:InverseTaskSingular', ...
            '逆任务求解遇到接近奇异的任务Jacobian。');
    end
    outputAngles = outputAngles + jacobian \ residual;
end

pose = cad_serial_leg_forward_kinematics(outputAngles, parameters);
residual = targetTask - [pose.legLength; pose.legAngle];
if norm(residual, inf) >= 1e-8
    error('sixdof:CadKinematics:InverseTaskFailed', ...
        '未能求得目标L0=%.6f m、腿角=%.6f rad。', targetTask);
end
result = packageResult(outputAngles, pose, residual, maximumIterations);
end

function result = packageResult(outputAngles, pose, residual, iterations)
result.outputAngles = outputAngles;
result.pose = pose;
result.residual = residual;
result.iterations = iterations;
end
