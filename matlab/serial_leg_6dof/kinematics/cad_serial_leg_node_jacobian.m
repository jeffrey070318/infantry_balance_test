function jacobian = cad_serial_leg_node_jacobian(outputAngles, nodeId, parameters)
%CAD_SERIAL_LEG_NODE_JACOBIAN Differentiate one CAD node against output angles.

arguments
    outputAngles (2, 1) double {mustBeFinite}
    nodeId (1, 1) double {mustBeInteger, mustBeInRange(nodeId, 1, 7)}
    parameters (1, 1) struct = cad_serial_leg_parameters()
end

step = 1e-7;
jacobian = zeros(2);
for column = 1:2
    delta = zeros(2, 1);
    delta(column) = step;
    plus = cad_serial_leg_forward_kinematics(outputAngles + delta, parameters);
    minus = cad_serial_leg_forward_kinematics(outputAngles - delta, parameters);
    jacobian(:, column) = ...
        (plus.nodes(:, nodeId) - minus.nodes(:, nodeId)) / (2 * step);
end
end
