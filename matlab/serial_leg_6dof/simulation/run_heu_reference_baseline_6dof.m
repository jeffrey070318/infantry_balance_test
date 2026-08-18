function result = run_heu_reference_baseline_6dof(makeFigure)
%RUN_HEU_REFERENCE_BASELINE_6DOF Run the system-level six-state baseline.

arguments
    makeFigure (1, 1) logical = true
end

parameters = heu_reference_parameters(0.18);
model = linearize_equivalent_leg_6dof(parameters);
controller = design_heu_lqr_6dof(model);
disturbance = simulate_initial_disturbance_6dof(model, controller);
schedule = build_heu_gain_schedule_6dof();

fprintf('A尺寸: %dx%d, B尺寸: %dx%d\n', size(model.A), size(model.B));
fprintf('可控性秩: %d\n', rank(ctrb(model.A, model.B)));
fprintf('最大闭环极点实部: %.6f\n', max(real(controller.closedLoopPoles)));
fprintf('非线性末姿态/速度无穷范数: %.3e\n', ...
    disturbance.finalAttitudeRateNorm);
fprintf('6秒末位置慢模态残差: %.3e m\n', disturbance.finalPositionError);
fprintf('三次增益拟合最大绝对误差: %.3e\n', ...
    max(schedule.maxAbsErrorByElement));
fprintf('三次增益拟合最大逐元素相对误差: %.3f%%\n', ...
    100 * max(schedule.maxRelativeErrorByElement));

result.parameters = parameters;
result.model = model;
result.controller = controller;
result.disturbance = disturbance;
result.schedule = schedule;
result.figurePath = "";

if makeFigure
    figureHandle = figure('Name', 'HEU six-state baseline', ...
        'Color', 'white', 'Position', [100, 100, 1200, 760]);
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(disturbance.time, rad2deg(disturbance.linearState(:, 5)), ...
        'LineWidth', 1.5);
    hold on;
    plot(disturbance.nonlinearTime, ...
        rad2deg(disturbance.nonlinearState(:, 5)), '--', 'LineWidth', 1.5);
    grid on;
    xlabel('时间 / s'); ylabel('机体俯仰角 / deg');
    title('2 deg初始扰动'); legend('线性模型', '非线性方程');

    nexttile;
    plot(disturbance.nonlinearTime, disturbance.nonlinearInput, 'LineWidth', 1.2);
    grid on;
    xlabel('时间 / s'); ylabel('控制输入 / N m');
    title('非线性闭环控制量'); legend('T', 'T_p');

    nexttile;
    semilogy(schedule.legLengths, ...
        max(abs(schedule.fitError), [], 2), 'o-', 'LineWidth', 1.2);
    grid on;
    xlabel('腿长 L_0 / m'); ylabel('12个K元素的最大拟合误差');
    title('哈工程三次增益拟合复算');

    nexttile;
    plot(schedule.legLengths, schedule.maxClosedLoopRealPart, ...
        'LineWidth', 1.5);
    yline(0, 'k--'); grid on;
    xlabel('腿长 L_0 / m'); ylabel('最大闭环极点实部');
    title('逐腿长重算LQR的冻结参数稳定性');

    resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
    if ~isfolder(resultDir), mkdir(resultDir); end
    result.figurePath = fullfile(resultDir, '10_heu_six_state_baseline.png');
    exportgraphics(figureHandle, result.figurePath, 'Resolution', 160);
    fprintf('曲线图已保存: %s\n', result.figurePath);
end
end
