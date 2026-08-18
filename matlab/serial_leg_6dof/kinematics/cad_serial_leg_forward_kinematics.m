function pose = cad_serial_leg_forward_kinematics(outputAngles, parameters)
%CAD_SERIAL_LEG_FORWARD_KINEMATICS Solve J1-J7 from two J1 output angles.

arguments
    outputAngles (2, 1) double {mustBeFinite}
    parameters (1, 1) struct = cad_serial_leg_parameters()
end

direction1 = [cos(outputAngles(1)); sin(outputAngles(1))];
direction2 = [cos(outputAngles(2)); sin(outputAngles(2))];
nodes = zeros(2, 7);
nodes(:, 1) = parameters.fixedPivot;
nodes(:, 2) = nodes(:, 1) + parameters.smallLegLength * direction1;
nodes(:, 4) = nodes(:, 1) + parameters.link2InputLength * direction2;
nodes(:, 3) = selectedCircleIntersection(nodes(:, 2), ...
    parameters.link1Length, nodes(:, 4), parameters.link5InputLength, ...
    parameters.firstLoopBranch);
nodes(:, 5) = selectedCircleIntersection(nodes(:, 4), ...
    parameters.link5ShortLength, nodes(:, 3), ...
    parameters.link5DiagonalLength, parameters.link5Branch);
nodes(:, 6) = nodes(:, 1) + parameters.link2TotalLength * direction2;
nodes(:, 7) = nodes(:, 6) + nodes(:, 5) - nodes(:, 4);

pose.nodes = nodes;
pose.outputAngles = outputAngles;
pose.distalReference = nodes(:, 7);
link4Unit = (nodes(:, 7) - nodes(:, 6)) / ...
    norm(nodes(:, 7) - nodes(:, 6));
link4Normal = [-link4Unit(2); link4Unit(1)];
pose.wheelCenter = nodes(:, 6) + ...
    parameters.wheelOffsetAlongLink4 * link4Unit + ...
    parameters.wheelOffsetNormalToLink4 * link4Normal;
pose.legVectorWheelToHip = nodes(:, 1) - pose.wheelCenter;
pose.legLength = norm(pose.legVectorWheelToHip);
pose.legAngle = atan2( ...
    dot(pose.legVectorWheelToHip, parameters.cadHorizontalAxis), ...
    dot(pose.legVectorWheelToHip, parameters.cadUpAxis));
pose.closureResidual = closureResidual(nodes, parameters);
pose.warning = parameters.warning;
end

function point = selectedCircleIntersection(center1, radius1, center2, radius2, branch)
baseline = center2 - center1;
centerDistance = norm(baseline);
tolerance = 1e-11;
if centerDistance < tolerance || centerDistance > radius1 + radius2 + tolerance || ...
        centerDistance < abs(radius1 - radius2) - tolerance
    error('sixdof:CadKinematics:Unreachable', ...
        '两个输出角使CAD闭环没有实数装配解。');
end
unitBaseline = baseline / centerDistance;
along = (radius1^2 - radius2^2 + centerDistance^2) / (2 * centerDistance);
heightSquared = radius1^2 - along^2;
height = sqrt(max(heightSquared, 0));
normal = [-unitBaseline(2); unitBaseline(1)];
point = center1 + along * unitBaseline + branch * height * normal;
end

function residual = closureResidual(nodes, p)
residual = [
    norm(nodes(:, 2) - nodes(:, 1)) - p.smallLegLength
    norm(nodes(:, 3) - nodes(:, 2)) - p.link1Length
    norm(nodes(:, 4) - nodes(:, 1)) - p.link2InputLength
    norm(nodes(:, 3) - nodes(:, 4)) - p.link5InputLength
    norm(nodes(:, 5) - nodes(:, 4)) - p.link5ShortLength
    norm(nodes(:, 5) - nodes(:, 3)) - p.link5DiagonalLength
    norm(nodes(:, 6) - nodes(:, 1)) - p.link2TotalLength
    norm(nodes(:, 7) - nodes(:, 6)) - p.link5ShortLength
    norm(nodes(:, 7) - nodes(:, 5)) - p.parallelogramLongLength
    nodes(:, 7) - nodes(:, 6) - nodes(:, 5) + nodes(:, 4)
    ];
end
