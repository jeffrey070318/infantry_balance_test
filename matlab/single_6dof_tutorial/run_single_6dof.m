function result = run_single_6dof(showPlots)
%RUN_SINGLE_6DOF Run the RM six-state continuous-time LQR tutorial.
%
% In the MATLAB desktop, change Current Folder to this file's directory and
% run:
%   run_single_6dof
%
% This is an RM reference reproduction, not a robot-specific model.

arguments
    showPlots (1, 1) logical = true
end

model = paper_model_6dof();
controller = design_lqr_6dof(model);

assert(isequal(size(model.A), [6 6]), 'A must be 6-by-6.');
assert(isequal(size(model.B), [6 2]), 'B must be 6-by-2.');
controllabilityRank = rank(ctrb(model.A, model.B));
assert(controllabilityRank == 6, 'The published model is not controllable.');
assert(all(real(controller.closedLoopPoles) < 0), ...
    'The LQR closed loop is not asymptotically stable.');

openLoopPoles = eig(model.A);
t = linspace(0, 10, 2001).';
x0 = zeros(6, 1);
x0(5) = deg2rad(2);

closedLoop = ss(model.A - model.B * controller.K, zeros(6, 1), ...
    eye(6), zeros(6, 1));
x = initial(closedLoop, x0, t);
u = -(controller.K * x.').';

fprintf('\n=== 哈工程 RM 六维参考模型：连续时间 LQR ===\n');
fprintf('%s\n', model.warning);
fprintf('A 尺寸: %d x %d\n', size(model.A));
fprintf('B 尺寸: %d x %d\n', size(model.B));
fprintf('可控性秩: %d / 6\n', controllabilityRank);
fprintf('开环最大极点实部: %.6f\n', max(real(openLoopPoles)));
fprintf('闭环最大极点实部: %.6f\n', max(real(controller.closedLoopPoles)));
fprintf('10 s 末最大状态绝对值: %.3e\n\n', norm(x(end, :), inf));

disp('LQR 增益 K（控制律 u = -K*x）:');
disp(controller.K);
disp('闭环极点:');
disp(controller.closedLoopPoles);

if showPlots
    figureHandle = plotSimulation(t, x, u, model);
    result.figurePath = save_figure_6dof( ...
        figureHandle, "01_initial_disturbance_lqr");
else
    result.figurePath = "";
end

result.model = model;
result.Q = controller.Q;
result.R = controller.R;
result.K = controller.K;
result.openLoopPoles = openLoopPoles;
result.closedLoopPoles = controller.closedLoopPoles;
result.controllabilityRank = controllabilityRank;
result.t = t;
result.x0 = x0;
result.x = x;
result.u = u;
end

function figureHandle = plotSimulation(t, x, u, model)
figureHandle = figure('Name', '单个六维线性模型 LQR', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, rad2deg(x(:, [1 5])), 'LineWidth', 1.3);
grid on;
ylabel('角度 (deg)');
legend('\theta 腿角', '\phi 机体俯仰', 'Location', 'best');
title('哈工程 RM 六维参考模型：2 deg 机体俯仰初始扰动');

nexttile;
yyaxis left;
plot(t, x(:, 3), 'LineWidth', 1.3);
ylabel('x_b (m)');
yyaxis right;
plot(t, x(:, 4), 'LineWidth', 1.3);
ylabel('dx_b/dt (m/s)');
grid on;
legend(model.stateNames(3), model.stateNames(4), 'Location', 'best');

nexttile;
plot(t, u, 'LineWidth', 1.3);
grid on;
xlabel('时间 (s)');
ylabel('输入 (N*m)');
legend('T 轮矩', 'T_p 等效摆矩', 'Location', 'best');
end
