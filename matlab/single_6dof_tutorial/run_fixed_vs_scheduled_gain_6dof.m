function result = run_fixed_vs_scheduled_gain_6dof(showPlots)
%RUN_FIXED_VS_SCHEDULED_GAIN_6DOF Compare fixed and scheduled K during leg motion.
%
% The plant uses a frozen-time A(L0)/B(L0) at each sample. This LPV
% approximation omits L0_dot and L0_ddot coupling, so it teaches gain
% scheduling but is not yet a complete extending-leg dynamic model.

arguments
    showPlots (1, 1) logical = true
end

sampleTime = 0.005;
t = (0:sampleTime:10).';
legLength = buildLegLengthProfile(t);
n = numel(t);

fixedModel = opensource_parameterized_model_6dof(0.18);
fixedK = design_opensource_lqr_6dof(fixedModel).K;

xFixed = zeros(n, 6);
xScheduled = zeros(n, 6);
uFixed = zeros(n, 2);
uScheduled = zeros(n, 2);
xFixed(1, 5) = deg2rad(2);
xScheduled(1, 5) = deg2rad(2);
disturbanceTimes = [4, 8];

for index = 1:n
    if any(abs(t(index) - disturbanceTimes) < sampleTime / 2)
        xFixed(index, 5) = xFixed(index, 5) + deg2rad(2);
        xScheduled(index, 5) = xScheduled(index, 5) + deg2rad(2);
    end

    model = opensource_parameterized_model_6dof(legLength(index));
    scheduledK = opensource_gain_schedule_6dof(legLength(index)).K;
    uFixed(index, :) = -(fixedK * xFixed(index, :).').';
    uScheduled(index, :) = -(scheduledK * xScheduled(index, :).').';

    if index < n
        discretePlant = c2d(ss(model.A, model.B, eye(6), zeros(6, 2)), ...
            sampleTime, 'zoh');
        xFixed(index + 1, :) = (discretePlant.A * xFixed(index, :).' + ...
            discretePlant.B * uFixed(index, :).').';
        xScheduled(index + 1, :) = ...
            (discretePlant.A * xScheduled(index, :).' + ...
            discretePlant.B * uScheduled(index, :).').';
    end
end

scanLengths = (0.12:0.01:0.36).';
fixedMaxReal = zeros(size(scanLengths));
scheduledMaxReal = zeros(size(scanLengths));
fixedMaxImag = zeros(size(scanLengths));
scheduledMaxImag = zeros(size(scanLengths));
for index = 1:numel(scanLengths)
    model = opensource_parameterized_model_6dof(scanLengths(index));
    scheduledK = opensource_gain_schedule_6dof(scanLengths(index)).K;
    fixedPoles = eig(model.A - model.B * fixedK);
    scheduledPoles = eig(model.A - model.B * scheduledK);
    fixedMaxReal(index) = max(real(fixedPoles));
    scheduledMaxReal(index) = max(real(scheduledPoles));
    fixedMaxImag(index) = max(abs(imag(fixedPoles)));
    scheduledMaxImag(index) = max(abs(imag(scheduledPoles)));
end

fixedPitchRms = rms(xFixed(:, 5));
scheduledPitchRms = rms(xScheduled(:, 5));
fixedControlEffort = sum(sum(uFixed.^2, 2)) * sampleTime;
scheduledControlEffort = sum(sum(uScheduled.^2, 2)) * sampleTime;
longLegWindow = t >= 4 & t <= 6;
shortLegWindow = t >= 8 & t <= 10;
longLegThetaRms = [rms(xFixed(longLegWindow, 1)), ...
    rms(xScheduled(longLegWindow, 1))];
shortLegThetaRms = [rms(xFixed(shortLegWindow, 1)), ...
    rms(xScheduled(shortLegWindow, 1))];
longLegWheelTorqueRms = [rms(uFixed(longLegWindow, 1)), ...
    rms(uScheduled(longLegWindow, 1))];

fprintf('\n=== 第二六维模型：固定K与K(L0)调度对比 ===\n');
fprintf('腿长轨迹: 0.18 -> 0.36 -> 0.12 m\n');
fprintf('机体俯仰扰动: 初始、4 s、8 s各 +2 deg\n');
fprintf('固定K机体俯仰RMS: %.6f deg\n', rad2deg(fixedPitchRms));
fprintf('调度K机体俯仰RMS: %.6f deg\n', rad2deg(scheduledPitchRms));
fprintf('固定K控制量平方积分: %.6f\n', fixedControlEffort);
fprintf('调度K控制量平方积分: %.6f\n', scheduledControlEffort);
fprintf('长腿段theta RMS 固定/调度: %.6f / %.6f deg\n', ...
    rad2deg(longLegThetaRms(1)), rad2deg(longLegThetaRms(2)));
fprintf('长腿段轮矩RMS 固定/调度: %.6f / %.6f N*m\n', ...
    longLegWheelTorqueRms(1), longLegWheelTorqueRms(2));
fprintf('短腿段theta RMS 固定/调度: %.6f / %.6f deg\n', ...
    rad2deg(shortLegThetaRms(1)), rad2deg(shortLegThetaRms(2)));
fprintf('固定K全腿长范围最大极点实部: %.6f\n', max(fixedMaxReal));
fprintf('调度K全腿长范围最大极点实部: %.6f\n\n', ...
    max(scheduledMaxReal));
fprintf(['注意: 当前是冻结参数LPV近似，尚未包含伸缩腿运动产生的' ...
    '惯性力与L0变化率耦合。\n\n']);

if showPlots
    figureHandle = plotComparison(t, legLength, xFixed, xScheduled, ...
        uFixed, uScheduled, scanLengths, fixedMaxImag, scheduledMaxImag);
    result.figurePath = save_figure_6dof( ...
        figureHandle, "09_fixed_vs_scheduled_gain");
else
    result.figurePath = "";
end

result.t = t;
result.legLength = legLength;
result.xFixed = xFixed;
result.xScheduled = xScheduled;
result.uFixed = uFixed;
result.uScheduled = uScheduled;
result.fixedPitchRms = fixedPitchRms;
result.scheduledPitchRms = scheduledPitchRms;
result.fixedControlEffort = fixedControlEffort;
result.scheduledControlEffort = scheduledControlEffort;
result.longLegThetaRms = longLegThetaRms;
result.shortLegThetaRms = shortLegThetaRms;
result.longLegWheelTorqueRms = longLegWheelTorqueRms;
result.scanLengths = scanLengths;
result.fixedMaxReal = fixedMaxReal;
result.scheduledMaxReal = scheduledMaxReal;
result.fixedMaxImag = fixedMaxImag;
result.scheduledMaxImag = scheduledMaxImag;
result.isFrozenParameterApproximation = true;
end

function legLength = buildLegLengthProfile(t)
legLength = zeros(size(t));
for index = 1:numel(t)
    if t(index) < 2
        legLength(index) = 0.18;
    elseif t(index) < 4
        legLength(index) = 0.18 + 0.18 * (t(index) - 2) / 2;
    elseif t(index) < 6
        legLength(index) = 0.36;
    elseif t(index) < 8
        legLength(index) = 0.36 - 0.24 * (t(index) - 6) / 2;
    else
        legLength(index) = 0.12;
    end
end
end

function figureHandle = plotComparison(t, legLength, xFixed, xScheduled, ...
    uFixed, uScheduled, scanLengths, fixedMaxImag, scheduledMaxImag)
figureHandle = figure('Name', '固定K与腿长调度K对比', 'Color', 'w');
tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, legLength, 'LineWidth', 1.3);
grid on;
ylabel('L0 (m)');
title('冻结参数LPV模型：固定K(0.18)与实时K(L0)');

nexttile;
plot(t, rad2deg(xFixed(:, 1)), '--', ...
    t, rad2deg(xScheduled(:, 1)), 'LineWidth', 1.2);
grid on;
ylabel('theta (deg)');
legend('固定K', '调度K', 'Location', 'best');

nexttile;
plot(t, uFixed(:, 1), '--', t, uScheduled(:, 1), 'LineWidth', 1.2);
grid on;
ylabel('轮矩T (N*m)');
legend('固定K', '调度K', 'Location', 'best');

nexttile;
plot(scanLengths, fixedMaxImag, '--o', ...
    scanLengths, scheduledMaxImag, '-o', 'LineWidth', 1.2, ...
    'MarkerSize', 3);
grid on;
xlabel('L0 (m)');
ylabel('最大|Im(pole)| (rad/s)');
legend('固定K闭环', '调度K闭环', 'Location', 'best');
end
