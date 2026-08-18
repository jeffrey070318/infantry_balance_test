function pose = closed_chain_forward_kinematics(jointAngles, geometry)
%CLOSED_CHAIN_FORWARD_KINEMATICS Solve the selected two-circle branch.

arguments
    jointAngles (2, 1) double {mustBeFinite}
    geometry (1, 1) struct
end

O = geometry.origin;
rotation90 = [0, -1; 1, 0];
A = O + geometry.activeLink1 * [cos(jointAngles(1)); sin(jointAngles(1))];
B = O + geometry.activeLink2 * [cos(jointAngles(2)); sin(jointAngles(2))];
baseline = B - A;
distance = norm(baseline);
b = geometry.passiveLink1;
c = geometry.passiveLink2;
tolerance = 1e-12;

if distance < tolerance || distance > b + c + tolerance || ...
        distance < abs(b - c) - tolerance
    error('sixdof:Kinematics:Unreachable', ...
        '当前关节角没有唯一可达的闭环交点。');
end

unitBaseline = baseline / distance;
along = (b^2 - c^2 + distance^2) / (2 * distance);
heightSquared = b^2 - along^2;
if heightSquared < -tolerance
    error('sixdof:Kinematics:Unreachable', '两圆没有实数交点。');
end
height = sqrt(max(heightSquared, 0));
C = A + along * unitBaseline + ...
    geometry.branch * height * rotation90 * unitBaseline;
relative = C - O;

pose.pointA = A;
pose.pointB = B;
pose.endpoint = C;
pose.legLength = norm(relative);
pose.legAngle = wrapToPiLocal(atan2(relative(2), relative(1)) - ...
    geometry.taskAngleZero);
pose.branchHeight = height;
pose.closureResidual = [norm(C - A) - b; norm(C - B) - c];
end

function angle = wrapToPiLocal(angle)
angle = mod(angle + pi, 2 * pi) - pi;
end
