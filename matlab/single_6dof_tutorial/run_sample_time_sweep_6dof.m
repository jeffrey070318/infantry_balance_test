function result = run_sample_time_sweep_6dof(showPlots)
%RUN_SAMPLE_TIME_SWEEP_6DOF Study stability versus firmware control period.
%
% The same continuous-time LQR gain K is executed with zero-order hold at
% different sample periods. For each period, discrete closed-loop stability
% is checked using rho(Ad - Bd*K) < 1.

arguments
    showPlots (1, 1) logical = true
end

model = paper_model_6dof();
controller = design_lqr_6dof(model);
sampleTimes = [0.001; 0.002; 0.005; 0.010; 0.015; 0.020; ...
    0.022; 0.024; 0.026; 0.028; 0.030];
n = numel(sampleTimes);

spectralRadius = zeros(n, 1);
maxLegAngleDeg = zeros(n, 1);
isEigenvalueStable = false(n, 1);
isSimulationDiverged = false(n, 1);
simulations = cell(n, 1);

for index = 1:n
    sampleTime = sampleTimes(index);
    t = (0:sampleTime:10).';
    reference = limited_speed_reference_6dof(t, 1.5, 3.0, 0.7);
    simulation = simulate_sampled_tracking_6dof( ...
        model, controller, reference, [Inf; Inf]);
    closedLoopMatrix = simulation.Ad - simulation.Bd * controller.K;

    spectralRadius(index) = max(abs(eig(closedLoopMatrix)));
    maxLegAngleDeg(index) = rad2deg(simulation.maxLegAngle);
    isEigenvalueStable(index) = spectralRadius(index) < 1;
    isSimulationDiverged(index) = simulation.isDiverged;
    simulations{index} = simulation;
end

sweepTable = table(1000 * sampleTimes, 1 ./ sampleTimes, ...
    spectralRadius, maxLegAngleDeg, isEigenvalueStable, ...
    isSimulationDiverged, 'VariableNames', {'PeriodMs', 'FrequencyHz', ...
    'SpectralRadius', 'MaxLegAngleDeg', 'EigenvalueStable', ...
    'SimulationDiverged'});

disp('=== 控制周期扫描：连续LQR增益直接用于离散控制 ===');
disp(sweepTable);

stableIndices = find(isEigenvalueStable);
unstableIndices = find(~isEigenvalueStable);
largestStablePeriod = max(sampleTimes(stableIndices));
firstUnstablePeriod = min(sampleTimes(unstableIndices));
fprintf('扫描中最大稳定周期: %.1f ms\n', 1000 * largestStablePeriod);
fprintf('扫描中最小不稳定周期: %.1f ms\n', 1000 * firstUnstablePeriod);
fprintf(['注意: 这只是无延迟、无丢包、精确状态反馈模型的理论结果，' ...
    '实机控制周期必须留出明显余量。\n\n']);

if showPlots
    figureHandle = plotSampleTimeSweep( ...
        sampleTimes, spectralRadius, simulations);
    result.figurePath = save_figure_6dof( ...
        figureHandle, "06_sample_time_sweep");
else
    result.figurePath = "";
end

result.table = sweepTable;
result.largestStablePeriod = largestStablePeriod;
result.firstUnstablePeriod = firstUnstablePeriod;
end

function figureHandle = plotSampleTimeSweep(sampleTimes, spectralRadius, simulations)
figureHandle = figure('Name', '六维模型控制周期扫描', 'Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(1000 * sampleTimes, spectralRadius, 'o-', 'LineWidth', 1.3);
yline(1, '--', '离散稳定边界');
grid on;
xlabel('控制周期 (ms)');
ylabel('最大特征值模');
title('rho(Ad - BdK) < 1 时离散闭环渐近稳定');

nexttile;
selectedPeriodsMs = [1, 20, 24, 30];
hold on;
for periodMs = selectedPeriodsMs
    index = find(abs(1000 * sampleTimes - periodMs) < 1e-9, 1);
    simulation = simulations{index};
    plot(simulation.t, rad2deg(simulation.x(:, 1)), ...
        'LineWidth', 1.2, 'DisplayName', sprintf('%d ms', periodMs));
end
yline(15, ':', '暂定小角度边界', 'HandleVisibility', 'off');
grid on;
xlabel('时间 (s)');
ylabel('腿角 theta (deg)');
legend('Location', 'best');
title('同一K在不同控制周期下的时域响应');
end
