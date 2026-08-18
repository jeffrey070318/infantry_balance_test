function result = run_parameterized_model_validation_6dof(showPlots)
%RUN_PARAMETERIZED_MODEL_VALIDATION_6DOF Validate the second six-state model.

arguments
    showPlots (1, 1) logical = true
end

samples = opensource_gain_samples_6dof();
n = numel(samples.legLength);
recomputedGains = zeros(n, 12);
controllabilityRanks = zeros(n, 1);
maxClosedLoopRealPart = zeros(n, 1);

for index = 1:n
    model = opensource_parameterized_model_6dof(samples.legLength(index));
    controller = design_opensource_lqr_6dof(model);
    recomputedGains(index, :) = [controller.K(1, :), controller.K(2, :)];
    controllabilityRanks(index) = rank(ctrb(model.A, model.B));
    maxClosedLoopRealPart(index) = max(real(controller.closedLoopPoles));
end

gainError = recomputedGains - samples.flatGains;
maxAbsGainError = max(abs(gainError), [], 'all');
worstIndex = find(abs(gainError) == maxAbsGainError, 1);
[worstLengthIndex, worstGainIndex] = ind2sub(size(gainError), worstIndex);

fprintf('\n=== 第二个六维模型：A(L0)/B(L0)重算验证 ===\n');
fprintf('验证腿长点数: %d\n', n);
fprintf('全部模型最小可控性秩: %d / 6\n', min(controllabilityRanks));
fprintf('全部闭环最大极点实部的最大值: %.6f\n', ...
    max(maxClosedLoopRealPart));
fprintf('重算K与开源25组样本的最大绝对误差: %.6f\n', maxAbsGainError);
fprintf('最大误差位置: L0=%.2f m, %s\n\n', ...
    samples.legLength(worstLengthIndex), samples.labels(worstGainIndex));

if showPlots
    figureHandle = plotValidation(samples, recomputedGains, gainError);
    result.figurePath = save_figure_6dof( ...
        figureHandle, "08_parameterized_model_validation");
else
    result.figurePath = "";
end

result.samples = samples;
result.recomputedGains = recomputedGains;
result.gainError = gainError;
result.maxAbsGainError = maxAbsGainError;
result.controllabilityRanks = controllabilityRanks;
result.maxClosedLoopRealPart = maxClosedLoopRealPart;
end

function figureHandle = plotValidation(samples, recomputedGains, gainError)
figureHandle = figure('Name', '第二个六维模型重算验证', 'Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(samples.legLength, samples.flatGains(:, [1, 5, 7, 11]), 'o', ...
    samples.legLength, recomputedGains(:, [1, 5, 7, 11]), '-', ...
    'LineWidth', 1.1, 'MarkerSize', 4);
grid on;
xlabel('L0 (m)');
ylabel('增益');
legend('K11样本', 'K15样本', 'K21样本', 'K25样本', ...
    'K11重算', 'K15重算', 'K21重算', 'K25重算', ...
    'Location', 'eastoutside');
title('代表性K元素：原始样本与A(L0)/B(L0)重算结果');

nexttile;
plot(samples.legLength, max(abs(gainError), [], 2), 'o-', ...
    'LineWidth', 1.2);
grid on;
xlabel('L0 (m)');
ylabel('该腿长最大|K误差|');
title('移植误差检查');
end
