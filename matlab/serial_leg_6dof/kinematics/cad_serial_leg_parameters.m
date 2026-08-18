function parameters = cad_serial_leg_parameters()
%CAD_SERIAL_LEG_PARAMETERS Derive two-output mechanism geometry from STEP axes.

cad = load_cad_serial_leg_geometry();
points = cad.planePointsM;
parameters.referenceNodes = points;
parameters.fixedPivot = points(:, 1);
parameters.smallLegLength = distance(points, 1, 2);
parameters.link1Length = distance(points, 2, 3);
parameters.link2InputLength = distance(points, 1, 4);
parameters.link5InputLength = distance(points, 4, 3);
parameters.link5ShortLength = distance(points, 4, 5);
parameters.link5DiagonalLength = distance(points, 3, 5);
parameters.link2TotalLength = distance(points, 1, 6);
parameters.parallelogramLongLength = distance(points, 5, 7);
parameters.referenceOutputAngles = [angleOf(points(:, 2) - points(:, 1)); ...
    angleOf(points(:, 4) - points(:, 1))];
parameters.firstLoopBranch = intersectionBranch( ...
    points(:, 2), points(:, 4), points(:, 3));
parameters.link5Branch = intersectionBranch( ...
    points(:, 4), points(:, 3), points(:, 5));
attachment = cad.rigidAttachmentMatches(1);
if ~strcmp(attachment.structural_component, '连杆4改A') || ...
        attachment.coincident_axis_count < 2
    error('sixdof:CadKinematics:WheelAttachmentUnverified', ...
        '轮轴与连杆4的刚性安装证据不足。');
end
parameters.wheelCenterReference = cad.wheelAxisPointM([2, 3]);
parameters.wheelRadius = cad.wheelRadiusM;
link4Vector = points(:, 7) - points(:, 6);
link4Unit = link4Vector / norm(link4Vector);
link4Normal = [-link4Unit(2); link4Unit(1)];
wheelOffset = parameters.wheelCenterReference - points(:, 6);
parameters.wheelOffsetAlongLink4 = dot(wheelOffset, link4Unit);
parameters.wheelOffsetNormalToLink4 = dot(wheelOffset, link4Normal);
parameters.cadUpAxis = [0; -1];
parameters.cadHorizontalAxis = [1; 0];
parameters.source = cad.source;
parameters.inputContract = ["q(1): J1到J2的小腿输出角"; ...
    "q(2): J1到J4的连杆2输出角"];
parameters.warning = "轮轴已确认刚接连杆4；输出角仍不是电机编码器角";
end

function value = distance(points, left, right)
value = norm(points(:, right) - points(:, left));
end

function angle = angleOf(vector)
angle = atan2(vector(2), vector(1));
end

function branch = intersectionBranch(center1, center2, selectedPoint)
baseline = center2 - center1;
normal = [-baseline(2); baseline(1)] / norm(baseline);
midlineProjection = dot(selectedPoint - center1, normal);
branch = sign(midlineProjection);
if branch == 0
    error('sixdof:CadKinematics:SingularReference', ...
        'CAD参考姿态的圆交点支路无法确定。');
end
end
