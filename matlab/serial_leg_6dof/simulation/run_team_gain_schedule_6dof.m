function result = run_team_gain_schedule_6dof(makeFigure)
%RUN_TEAM_GAIN_SCHEDULE_6DOF Validate provisional K(L0) and online switching.

arguments
    makeFigure (1, 1) logical = true
end

firmware = team_firmware_parameters();
schedule = build_team_gain_schedule_6dof(19, ...
    firmware.legLengthRange(1):0.01:firmware.legLengthRange(2));
modes = ["scheduled", "fixed"];
simulations = cell(numel(modes), 1);
for index = 1:numel(modes)
    simulations{index} = simulate_scheduled_leg_length_6dof( ...
        schedule, modes(index), deg2rad(10), 0.001, 4, ...
        [0, 1.5, 3], [0.15, 0.32, 0.30], ...
        firmware.legLengthDefault, firmware.maxLegLengthRate, 1);
end

fprintf('本队暂估K(L0): 19 kg, L0=%.2f~%.2f m\n', schedule.validRange);
fprintf('  三次拟合最大逐元素相对误差: %.4f%%\n', ...
    100 * max(schedule.maxRelativeErrorByElement));
fprintf('  拟合K冻结点最差闭环极点实部: %.6f\n', ...
    max(schedule.fittedMaxRealPart));
for index = 1:numel(modes)
    simulation = simulations{index};
    fprintf(['  %s: 收敛=%d, 最大腿长速度=%.4f m/s, ' ...
        '关节饱和占比=%.3f%%, 峰值[T,Tp]=[%.3f, %.3f] N*m\n'], ...
        modes(index), simulation.isConverged, ...
        simulation.maxActualLegLengthRate, ...
        100 * simulation.jointSaturationFraction, simulation.maxRawInputNm);
end

result.schedule = schedule;
result.modes = modes;
result.simulations = simulations;
result.warning = [
    "K(L0)由19 kg暂估模型生成，不能直接下发实机"
    "在线切换验证为六维准静态LPV，不含主动伸缩动力学耦合"
    ];
result.figurePath = "";

if makeFigure
    colors = lines(numel(modes));
    labels = ["本队K(L0)", "固定L0=0.30 m的K"];
    figureHandle = figure('Name', 'Team gain schedule', ...
        'Color', 'white', 'Position', [90, 90, 1400, 850]);
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    hold on;
    for index = 1:numel(modes)
        plot(simulations{index}.time, ...
            rad2deg(simulations{index}.state(:, 5)), ...
            'LineWidth', 1.4, 'Color', colors(index, :));
    end
    grid on; xlabel('时间 / s'); ylabel('机体俯仰角 / deg');
    title('10 deg扰动下的腿长往返切换'); legend(labels, 'Location', 'best');

    nexttile;
    plot(simulations{1}.time, simulations{1}.targetLength, ...
        'k--', 'LineWidth', 1.2); hold on;
    plot(simulations{1}.time, simulations{1}.legLength, ...
        'LineWidth', 1.5, 'Color', colors(1, :));
    yline(schedule.validRange(1), ':');
    yline(schedule.validRange(2), ':');
    grid on; xlabel('时间 / s'); ylabel('L0 / m');
    title('0.2 m/s限速后的实际腿长'); legend('目标', '实际', 'Location', 'best');

    nexttile;
    plot(schedule.legLengths, schedule.exactMaxRealPart, ...
        'o-', 'LineWidth', 1.3); hold on;
    plot(schedule.legLengths, schedule.fittedMaxRealPart, ...
        'x--', 'LineWidth', 1.3);
    yline(0, 'k:'); grid on;
    xlabel('冻结腿长 L0 / m'); ylabel('最大闭环极点实部');
    title('逐点重算K与三次拟合K'); legend('逐点LQR', '三次拟合', 'Location', 'best');

    nexttile;
    hold on;
    for index = 1:numel(modes)
        plot(simulations{index}.time, simulations{index}.gainNorm, ...
            'LineWidth', 1.4, 'Color', colors(index, :));
    end
    grid on; xlabel('时间 / s'); ylabel('||K||_F');
    title('在线增益变化'); legend(labels, 'Location', 'best');

    resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
    if ~isfolder(resultDir), mkdir(resultDir); end
    result.figurePath = fullfile(resultDir, '23_team_gain_schedule.png');
    exportgraphics(figureHandle, result.figurePath, 'Resolution', 160);
    fprintf('本队K(L0)验证图已保存: %s\n', result.figurePath);
end
end
