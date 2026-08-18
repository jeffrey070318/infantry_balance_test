function result = run_speed_tracking_6dof(showPlots)
%RUN_SPEED_TRACKING_6DOF Reproduce the RM reference velocity scenario.
%
% The desired speed is 1.5 m/s before 3 s and zero afterwards. Desired
% position is its integral. The feedback law is u = K*(xd - x).

arguments
    showPlots (1, 1) logical = true
end

model = paper_model_6dof();
controller = design_lqr_6dof(model);

t = (0:0.005:8).';
reference = speed_reference_6dof(t, 1.5, 3.0);

simulation = simulate_tracking_6dof(model, controller, reference);
x = simulation.x;
u = simulation.u;

fprintf('\n=== 哈工程 RM 六维参考模型：速度跟踪 ===\n');
fprintf('速度指令: %.2f m/s, 0 < t < %.2f s\n', ...
    reference.targetSpeed, reference.stopTime);
fprintf('期望停车位置: %.3f m\n', reference.position(end));
fprintf('8 s 实际位置: %.3f m\n', x(end, 3));
fprintf('8 s 实际速度: %.3e m/s\n', x(end, 4));
fprintf('最大腿角绝对值: %.3f deg\n', rad2deg(simulation.maxLegAngle));
fprintf('最大机体俯仰绝对值: %.3f deg\n', ...
    rad2deg(simulation.maxBodyPitch));
fprintf('最大轮矩绝对值: %.3f N*m\n', max(abs(u(:, 1))));
fprintf('最大等效摆矩绝对值: %.3f N*m\n\n', max(abs(u(:, 2))));
if ~simulation.isWithinSmallAngleRange
    fprintf(['注意: 理想速度阶跃使姿态超过 +/-15 deg，本结果用于展示' ...
        '参考公式，不应直接作为实机速度指令。\n\n']);
end

if showPlots
    figureHandle = plotSpeedTracking(t, reference, x, u);
    result.figurePath = save_figure_6dof( ...
        figureHandle, "02_ideal_speed_step");
else
    result.figurePath = "";
end

result.model = model;
result.Q = controller.Q;
result.R = controller.R;
result.K = controller.K;
result.t = simulation.t;
result.xd = simulation.xd;
result.x = simulation.x;
result.u = simulation.u;
result.positionError = simulation.positionError;
result.velocityError = simulation.velocityError;
result.maxLegAngle = simulation.maxLegAngle;
result.maxBodyPitch = simulation.maxBodyPitch;
result.maxWheelTorque = simulation.maxWheelTorque;
result.maxPitchTorque = simulation.maxPitchTorque;
result.smallAngleLimit = simulation.smallAngleLimit;
result.isWithinSmallAngleRange = simulation.isWithinSmallAngleRange;
end

function figureHandle = plotSpeedTracking(t, reference, x, u)
figureHandle = figure('Name', '哈工程 RM 六维模型速度跟踪', 'Color', 'w');
tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, reference.velocity, '--', t, x(:, 4), 'LineWidth', 1.3);
grid on;
ylabel('速度 (m/s)');
legend('期望速度', '实际速度', 'Location', 'best');
title('《Robomaster平衡机器人系统控制-韭菜的菜》速度工况');

nexttile;
plot(t, reference.position, '--', t, x(:, 3), 'LineWidth', 1.3);
grid on;
ylabel('位置 (m)');
legend('期望位置', '实际位置', 'Location', 'best');

nexttile;
plot(t, rad2deg(x(:, [1 5])), 'LineWidth', 1.3);
grid on;
ylabel('角度 (deg)');
legend('\theta 腿角', '\phi 机体俯仰', 'Location', 'best');

nexttile;
plot(t, u, 'LineWidth', 1.3);
grid on;
xlabel('时间 (s)');
ylabel('输入 (N*m)');
legend('T 轮矩', 'T_p 等效摆矩', 'Location', 'best');
end
