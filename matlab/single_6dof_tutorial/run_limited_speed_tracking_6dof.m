function result = run_limited_speed_tracking_6dof(showPlots)
%RUN_LIMITED_SPEED_TRACKING_6DOF Compare ideal and acceleration-limited commands.

arguments
    showPlots (1, 1) logical = true
end

model = paper_model_6dof();
controller = design_lqr_6dof(model);
t = (0:0.005:10).';

idealReference = speed_reference_6dof(t, 1.5, 3.0);
% 0.7 m/s^2 is a teaching choice obtained from run_acceleration_sweep_6dof.
% It keeps this reference response below the provisional 15 deg limit.
limitedReference = limited_speed_reference_6dof(t, 1.5, 3.0, 0.7);
ideal = simulate_tracking_6dof(model, controller, idealReference);
limited = simulate_tracking_6dof(model, controller, limitedReference);

fprintf('\n=== 同一六维模型：只改变速度指令形状 ===\n');
fprintf('理想阶跃最大轮矩:       %8.3f N*m\n', ideal.maxWheelTorque);
fprintf('限加速度指令最大轮矩:   %8.3f N*m\n', limited.maxWheelTorque);
fprintf('理想阶跃最大腿角:       %8.3f deg\n', rad2deg(ideal.maxLegAngle));
fprintf('限加速度指令最大腿角:   %8.3f deg\n', rad2deg(limited.maxLegAngle));
fprintf('限加速度值:             %8.3f m/s^2\n', ...
    limitedReference.maxAcceleration);
fprintf('限加速度指令停车时刻:   %8.3f s\n', limitedReference.stopTime);
fprintf('两种指令最终目标位置:   %8.3f m\n\n', ...
    limitedReference.finalPosition);

if limited.isWithinSmallAngleRange
    fprintf(['当前参考响应处于暂定的 +/-15 deg 小角度范围内。' ...
        '这不是实机安全认证，换参数后必须重新扫描。\n\n']);
else
    fprintf(['注意: 限加速度后仍超过线性模型的小角度范围，' ...
        '应继续减小加速度或加入输入约束。\n\n']);
end

if showPlots
    figureHandle = plotComparison( ...
        idealReference, limitedReference, ideal, limited);
    result.figurePath = save_figure_6dof( ...
        figureHandle, "03_ideal_vs_limited_reference");
else
    result.figurePath = "";
end

result.model = model;
result.K = controller.K;
result.idealReference = idealReference;
result.limitedReference = limitedReference;
result.ideal = ideal;
result.limited = limited;
end

function figureHandle = plotComparison(idealReference, limitedReference, ideal, limited)
figureHandle = figure('Name', ...
    '理想阶跃与限加速度速度指令对比', 'Color', 'w');
tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(ideal.t, idealReference.velocity, '--', ...
    limited.t, limitedReference.velocity, 'LineWidth', 1.3);
grid on;
ylabel('期望速度 (m/s)');
legend('资料理想阶跃', '工程限加速度', 'Location', 'best');
title('A、B、K 均不变，仅改变参考指令');

nexttile;
plot(ideal.t, ideal.x(:, 4), '--', limited.t, limited.x(:, 4), ...
    'LineWidth', 1.3);
grid on;
ylabel('实际速度 (m/s)');
legend('阶跃响应', '限加速度响应', 'Location', 'best');

nexttile;
plot(ideal.t, rad2deg(ideal.x(:, 1)), '--', ...
    limited.t, rad2deg(limited.x(:, 1)), 'LineWidth', 1.3);
grid on;
ylabel('腿角 theta (deg)');
legend('阶跃响应', '限加速度响应', 'Location', 'best');

nexttile;
plot(ideal.t, ideal.u(:, 1), '--', limited.t, limited.u(:, 1), ...
    'LineWidth', 1.3);
grid on;
xlabel('时间 (s)');
ylabel('轮矩 T (N*m)');
legend('阶跃响应', '限加速度响应', 'Location', 'best');
end
