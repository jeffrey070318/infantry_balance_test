function [jacobian, diagnostics] = cad_serial_leg_task_jacobian(outputAngles, parameters)
%CAD_SERIAL_LEG_TASK_JACOBIAN Map output rates to [legLength; legAngle] rates.

arguments
    outputAngles (2, 1) double {mustBeFinite}
    parameters (1, 1) struct = cad_serial_leg_parameters()
end

step = 1e-7;
jacobian = zeros(2);
for column = 1:2
    delta = zeros(2, 1);
    delta(column) = step;
    plus = cad_serial_leg_forward_kinematics(outputAngles + delta, parameters);
    minus = cad_serial_leg_forward_kinematics(outputAngles - delta, parameters);
    jacobian(:, column) = [
        (plus.legLength - minus.legLength) / (2 * step)
        wrappedDifference(plus.legAngle, minus.legAngle) / (2 * step)
        ];
end

diagnostics.pose = cad_serial_leg_forward_kinematics(outputAngles, parameters);
diagnostics.determinant = det(jacobian);
diagnostics.conditionNumber = cond(jacobian);
end

function difference = wrappedDifference(left, right)
difference = mod(left - right + pi, 2 * pi) - pi;
end
