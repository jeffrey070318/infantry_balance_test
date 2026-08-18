function result = run_gain_schedule_method_6dof(showPlots)
%RUN_GAIN_SCHEDULE_METHOD_6DOF Reproduce an RM K-versus-leg-length fit.
%
% This is a method demonstration from a separate parameterized RM model.
% It does not turn paper_model_6dof into a variable-leg-length model.

arguments
    showPlots (1, 1) logical = true
end

samples = opensource_gain_samples_6dof();
fittedAtSamples = opensource_gain_schedule_6dof(samples.legLength);
fitError = fittedAtSamples.flatGains - samples.flatGains;
maxAbsError = max(abs(fitError), [], 1);

openSourceAt018 = opensource_gain_schedule_6dof(0.18);
paperController = design_lqr_6dof(paper_model_6dof());
differenceAt018 = openSourceAt018.K - paperController.K;

fprintf('\n=== RM开源增益拟合方法：K(L0) ===\n');
fprintf('开源样本腿长范围: %.2f ~ %.2f m\n', ...
    samples.legLength(1), samples.legLength(end));
fprintf('样本数量: %d\n', numel(samples.legLength));
fprintf('三次多项式对12个增益的最大绝对拟合误差: %.6f\n', ...
    max(maxAbsError));
fprintf('两套K在 L0=0.18 m 处的最大元素差: %.6f\n\n', ...
    max(abs(differenceAt018), [], 'all'));
fprintf(['结论: 这套K(L0)只用于学习增益调度方法，不能控制当前' ...
    '《韭菜的菜》固定腿长模型。\n\n']);

if showPlots
    figureHandle = plotGainFits(samples);
    result.figurePath = save_figure_6dof( ...
        figureHandle, "07_open_source_gain_schedule_method");
else
    result.figurePath = "";
end

result.samples = samples;
result.fittedAtSamples = fittedAtSamples;
result.maxAbsError = maxAbsError;
result.openSourceAt018 = openSourceAt018.K;
result.paperAt018 = paperController.K;
result.differenceAt018 = differenceAt018;
result.isCompatibleWithPaperModel = false;
end

function figureHandle = plotGainFits(samples)
denseLength = linspace(0.12, 0.36, 241).';
denseFit = opensource_gain_schedule_6dof(denseLength);
figureHandle = figure('Name', 'RM开源K与腿长拟合方法', 'Color', 'w');
tiledlayout(3, 4, 'TileSpacing', 'compact', 'Padding', 'compact');

for gainIndex = 1:12
    nexttile;
    plot(samples.legLength, samples.flatGains(:, gainIndex), 'o', ...
        denseLength, denseFit.flatGains(:, gainIndex), '-', ...
        'LineWidth', 1.1, 'MarkerSize', 3);
    grid on;
    title(samples.labels(gainIndex));
    xlabel('L0 (m)');
    ylabel('增益');
    if gainIndex == 1
        legend('离散LQR样本', '三次拟合', 'Location', 'best');
    end
end
sgtitle('独立RM开源模型的K(L0)方法演示，不可用于当前主模型');
end
