function geometry = load_cad_serial_leg_geometry(jsonPath)
%LOAD_CAD_SERIAL_LEG_GEOMETRY Load joint-axis candidates extracted from STEP.

arguments
    jsonPath (1, 1) string = defaultJsonPath()
end

if ~isfile(jsonPath)
    error('sixdof:CadGeometry:FileNotFound', ...
        '没有找到STEP提取结果: %s', jsonPath);
end

payload = jsondecode(fileread(jsonPath));
if ~strcmp(payload.status, 'candidate_only_requires_manual_confirmation')
    error('sixdof:CadGeometry:UnexpectedStatus', ...
        'STEP提取结果状态未知，拒绝当作候选几何载入。');
end

candidateCount = numel(payload.candidates);
pointsMm = zeros(3, candidateCount);
directions = zeros(3, candidateCount);
components = cell(candidateCount, 1);
for index = 1:candidateCount
    candidate = payload.candidates(index);
    if candidate.candidate_id ~= index
        error('sixdof:CadGeometry:CandidateOrder', ...
            '候选轴编号必须从1连续排列。');
    end
    pointsMm(:, index) = candidate.point_mm(:);
    directions(:, index) = candidate.direction(:);
    components{index} = string(candidate.components(:));
end

geometry.source = jsonPath;
geometry.status = string(payload.status);
geometry.pointsMm = pointsMm;
geometry.pointsM = pointsMm / 1000;
geometry.planePointsM = pointsMm([2, 3], :) / 1000;
geometry.axisDirections = directions;
geometry.components = components;
geometry.linkAxisDistances = payload.same_component_axis_distances;
geometry.missingRootTransforms = string(payload.components_without_root_transform);
wheelEvidence = payload.wheel_attachment_evidence;
geometry.wheelAxisPointMm = wheelEvidence.wheel_axis_point_mm(:);
geometry.wheelAxisPointM = geometry.wheelAxisPointMm / 1000;
geometry.wheelRadiusM = wheelEvidence.wheel_outer_radius_mm / 1000;
geometry.rigidAttachmentMatches = wheelEvidence.rigid_attachment_matches;
geometry.sprocketGeometryEvidence = payload.sprocket_geometry_evidence;
geometry.warning = "CAD轴距已提取，但链轮/齿轮约束、主动轴、零位和正方向尚未确认";
end

function jsonPath = defaultJsonPath()
projectRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
jsonPath = string(fullfile(projectRoot, 'cad', 'serial_leg', ...
    'extracted', 'joint_axis_candidates.json'));
end
