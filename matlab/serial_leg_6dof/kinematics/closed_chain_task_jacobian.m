function [jacobian, diagnostics] = closed_chain_task_jacobian(jointAngles, geometry)
%CLOSED_CHAIN_TASK_JACOBIAN Map joint rates to [L_dot; theta_dot].

pose = closed_chain_forward_kinematics(jointAngles, geometry);
O = geometry.origin;
A = pose.pointA;
B = pose.pointB;
C = pose.endpoint;
rotation90 = [0, -1; 1, 0];
r1 = C - A;
r2 = C - B;

constraintMatrix = [r1.'; r2.'];
driveMatrix = diag([r1.' * rotation90 * (A - O), ...
    r2.' * rotation90 * (B - O)]);
if rcond(constraintMatrix) < 1e-10
    error('sixdof:Kinematics:ConstraintSingularity', ...
        '闭环约束Jacobian接近奇异。');
end

cartesianJacobian = constraintMatrix \ driveMatrix;
relative = C - O;
L = pose.legLength;
polarJacobian = [relative(1) / L, relative(2) / L; ...
    -relative(2) / L^2, relative(1) / L^2];
jacobian = polarJacobian * cartesianJacobian;

diagnostics.pose = pose;
diagnostics.cartesianJacobian = cartesianJacobian;
diagnostics.constraintDeterminant = det(constraintMatrix);
diagnostics.driveLevers = diag(driveMatrix);
diagnostics.taskDeterminant = det(jacobian);
diagnostics.taskCondition = cond(jacobian);
end
