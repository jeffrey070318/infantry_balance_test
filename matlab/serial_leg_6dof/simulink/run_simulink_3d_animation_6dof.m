function result = run_simulink_3d_animation_6dof(showWindow)
%RUN_SIMULINK_3D_ANIMATION_6DOF Animate a simplified robot from Simulink states.

arguments
    showWindow (1, 1) logical = true
end

startup_serial_leg_6dof();
modelPath = build_serial_leg_6dof_model();
[~, modelName] = fileparts(modelPath);
load_system(modelPath);
simulationOutput = sim(modelName, 'ReturnWorkspaceOutputs', 'on');
state = simulationOutput.state_6dof;
close_system(modelName, 0);

frameRate = 30;
frameTime = (0:1/frameRate:state.Time(end)).';
frameState = interp1(state.Time, state.Data, frameTime, 'linear');
rootDir = fileparts(fileparts(mfilename('fullpath')));
resultsDir = fullfile(rootDir, 'results');
videoPath = fullfile(resultsDir, '27_simulink_3d_motion.mp4');
previewPath = fullfile(resultsDir, '27_simulink_3d_motion_preview.png');
writer = VideoWriter(videoPath, 'MPEG-4');
writer.FrameRate = frameRate;
writer.Quality = 95;
open(writer);
cleanupWriter = onCleanup(@() close(writer));

visibility = 'off';
if showWindow, visibility = 'on'; end
figureHandle = figure('Name', 'Simulink 3D wheel-legged robot', ...
    'Color', [0.96, 0.97, 0.98], 'Position', [120, 80, 1100, 760], ...
    'Visible', visibility);
axesHandle = axes(figureHandle);

for index = 1:numel(frameTime)
    drawRobotFrame(axesHandle, frameState(index, :), frameTime(index));
    drawnow;
    if index == 1
        exportgraphics(axesHandle, previewPath, 'Resolution', 160);
    end
    writeVideo(writer, getframe(figureHandle));
    if showWindow
        pause(1 / frameRate);
    end
end
close(writer);
clear cleanupWriter;

result.modelPath = modelPath;
result.videoPath = videoPath;
result.previewPath = previewPath;
result.frameRate = frameRate;
result.duration = frameTime(end);
result.warning = "三维实体由六维状态驱动，仅作可视化，不含轮地接触物理。";
fprintf('三维运动视频: %s\n', videoPath);
end

function drawRobotFrame(axesHandle, state, time)
cla(axesHandle);
hold(axesHandle, 'on');

legAngle = state(1);
position = state(3);
bodyPitch = state(5);
wheelRadius = 0.06;
legLength = 0.30;
halfTrack = 0.2175;
wheelWidth = 0.045;

surf(axesHandle, [-0.6, 0.6; -0.6, 0.6], ...
    [-0.42, -0.42; 0.42, 0.42], zeros(2), ...
    'FaceColor', [0.72, 0.75, 0.78], 'EdgeColor', 'none');

wheelCenters = [position, -halfTrack, wheelRadius; ...
    position, halfTrack, wheelRadius];
hipX = position + legLength * sin(legAngle);
hipZ = wheelRadius + legLength * cos(legAngle);
hipPoints = [hipX, -halfTrack, hipZ; hipX, halfTrack, hipZ];

for side = 1:2
    drawWheel(axesHandle, wheelCenters(side, :), wheelRadius, wheelWidth, ...
        -position / wheelRadius);
    plot3(axesHandle, [wheelCenters(side, 1), hipPoints(side, 1)], ...
        [wheelCenters(side, 2), hipPoints(side, 2)], ...
        [wheelCenters(side, 3), hipPoints(side, 3)], ...
        'Color', [0.12, 0.16, 0.20], 'LineWidth', 9);
    plot3(axesHandle, [wheelCenters(side, 1) - 0.035, hipPoints(side, 1) + 0.035], ...
        [wheelCenters(side, 2), hipPoints(side, 2)], ...
        [wheelCenters(side, 3), hipPoints(side, 3)], ...
        'Color', [0.10, 0.55, 0.62], 'LineWidth', 4);
end

bodyCenter = [hipX + 0.10 * sin(bodyPitch), 0, ...
    hipZ + 0.10 * cos(bodyPitch)];
drawBox(axesHandle, bodyCenter, [0.28, 0.36, 0.20], bodyPitch);
plot3(axesHandle, hipPoints(:, 1), hipPoints(:, 2), hipPoints(:, 3), ...
    'Color', [0.20, 0.23, 0.27], 'LineWidth', 7);

axis(axesHandle, 'equal');
xlim(axesHandle, [-0.5, 0.5]);
ylim(axesHandle, [-0.42, 0.42]);
zlim(axesHandle, [0, 0.62]);
view(axesHandle, 36, 20);
grid(axesHandle, 'on');
xlabel(axesHandle, '前进方向 x / m');
ylabel(axesHandle, '横向 y / m');
zlabel(axesHandle, '高度 z / m');
title(axesHandle, sprintf('t = %.2f s    机体俯仰 = %.2f deg', ...
    time, rad2deg(bodyPitch)));
set(axesHandle, 'Color', [0.90, 0.94, 0.97], 'Projection', 'perspective');
camlight(axesHandle, 'headlight');
lighting(axesHandle, 'gouraud');
end

function drawWheel(axesHandle, center, radius, width, rotation)
[circleX, circleY, axial] = cylinder(radius, 28);
wheelX = center(1) + circleX;
wheelY = center(2) + width * (axial - 0.5);
wheelZ = center(3) + circleY;
surf(axesHandle, wheelX, wheelY, wheelZ, ...
    'FaceColor', [0.08, 0.09, 0.10], 'EdgeColor', 'none');
outerY = center(2) + sign(center(2)) * width / 2;
for offset = [0, pi / 2]
    angle = rotation + offset;
    plot3(axesHandle, center(1) + radius * [-cos(angle), cos(angle)], ...
        [outerY, outerY], center(3) + radius * [-sin(angle), sin(angle)], ...
        'Color', [0.75, 0.78, 0.80], 'LineWidth', 2);
end
end

function drawBox(axesHandle, center, dimensions, pitch)
half = dimensions / 2;
vertices = [
    -half(1), -half(2), -half(3)
     half(1), -half(2), -half(3)
     half(1),  half(2), -half(3)
    -half(1),  half(2), -half(3)
    -half(1), -half(2),  half(3)
     half(1), -half(2),  half(3)
     half(1),  half(2),  half(3)
    -half(1),  half(2),  half(3)
    ];
rotation = [cos(pitch), 0, sin(pitch); 0, 1, 0; ...
    -sin(pitch), 0, cos(pitch)];
vertices = vertices * rotation.' + center;
faces = [1, 2, 3, 4; 5, 8, 7, 6; 1, 5, 6, 2; ...
    2, 6, 7, 3; 3, 7, 8, 4; 4, 8, 5, 1];
patch(axesHandle, 'Vertices', vertices, 'Faces', faces, ...
    'FaceColor', [0.86, 0.20, 0.16], 'FaceAlpha', 0.92, ...
    'EdgeColor', [0.35, 0.08, 0.06], 'LineWidth', 1.1);
end
