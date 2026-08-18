function result = run_firmware_gain_comparison_6dof(makeFigure)
%RUN_FIRMWARE_GAIN_COMPARISON_6DOF Compare firmware K with provisional LQR.

arguments
    makeFigure (1, 1) logical = true
end

firmware = team_firmware_parameters();
legLengths = (firmware.legLengthRange(1):0.01:firmware.legLengthRange(2)).';
totalMasses = [18, 19, 20];
firmwareMaxRealPart = zeros(numel(legLengths), numel(totalMasses));
recomputedMaxRealPart = zeros(size(firmwareMaxRealPart));
gainRelativeDifference = zeros(size(firmwareMaxRealPart));

for lengthIndex = 1:numel(legLengths)
    firmwareGain = firmware_gain_schedule_6dof(legLengths(lengthIndex));
    for massIndex = 1:numel(totalMasses)
        parameters = team_estimated_dynamics_parameters( ...
            totalMasses(massIndex), legLengths(lengthIndex));
        model = linearize_equivalent_leg_6dof(parameters);
        recomputed = design_heu_lqr_6dof(model);
        firmwareMaxRealPart(lengthIndex, massIndex) = max(real(eig( ...
            model.A - model.B * firmwareGain.K)));
        recomputedMaxRealPart(lengthIndex, massIndex) = max(real( ...
            recomputed.closedLoopPoles));
        gainRelativeDifference(lengthIndex, massIndex) = norm( ...
            firmwareGain.K - recomputed.K, 'fro') / norm(recomputed.K, 'fro');
    end
end

nominalLength = firmware.legLengthDefault;
nominalParameters = team_estimated_dynamics_parameters(19, nominalLength);
nominalModel = linearize_equivalent_leg_6dof(nominalParameters);
recomputedController = design_heu_lqr_6dof(nominalModel);
firmwareSchedule = firmware_gain_schedule_6dof(nominalLength);
firmwareController.K = firmwareSchedule.K;
firmwareController.closedLoopPoles = eig( ...
    nominalModel.A - nominalModel.B * firmwareController.K);
firmwareController.feedbackLaw = "u=-K*x after MATLAB sign audit";

recomputedDisturbance = simulate_initial_disturbance_6dof( ...
    nominalModel, recomputedController);
firmwareDisturbance = simulate_initial_disturbance_6dof( ...
    nominalModel, firmwareController);
recomputedPeakInput = max(abs(recomputedDisturbance.nonlinearInput), [], 1);
firmwarePeakInput = max(abs(firmwareDisturbance.nonlinearInput), [], 1);
firmwarePitchMetrics = decayMetrics( ...
    firmwareDisturbance.nonlinearTime, firmwareDisturbance.nonlinearState(:, 5));
recomputedPitchMetrics = decayMetrics( ...
    recomputedDisturbance.nonlinearTime, recomputedDisturbance.nonlinearState(:, 5));

cad = cad_serial_leg_parameters();
cadPose = cad_serial_leg_forward_kinematics(cad.referenceOutputAngles, cad);
cadFirmwareSchedule = firmware_gain_schedule_6dof(cadPose.legLength);
cadFirmwareController.K = cadFirmwareSchedule.K;
stateAtCad = zeros(6, 1);
stateAtCad(5) = deg2rad(2);
configuration.axialKpNPerM = 0;
configuration.axialKdNsPerM = 0;
configuration.axialFeedforwardN = ...
    nominalParameters.totalMass * nominalParameters.gravity / ...
    (2 * cos(cadPose.legAngle));
configuration.limitMode = "rated";
cadControlCommand = serial_leg_control_step_6dof( ...
    stateAtCad, zeros(6, 1), [cadPose.legLength; 0], ...
    [cadPose.legLength; 0], cad.referenceOutputAngles, ...
    cadFirmwareController, configuration, cad);

fprintf('同构固件活动K复核，L0=%.3f m:\n', nominalLength);
disp(firmwareSchedule.K);
fprintf('  固件K在当前模型的最大闭环极点实部: %.6f\n', ...
    max(real(firmwareController.closedLoopPoles)));
fprintf('  当前重算K的最大闭环极点实部: %.6f\n', ...
    max(real(recomputedController.closedLoopPoles)));
fprintf('  全腿长/质量扫描中固件K最差极点实部: %.6f\n', ...
    max(firmwareMaxRealPart, [], 'all'));
fprintf('  名义2 deg扰动峰值[T,Tp]，固件K=[%.3f, %.3f] N*m\n', ...
    firmwarePeakInput);
fprintf('  名义2 deg扰动峰值[T,Tp]，重算K=[%.3f, %.3f] N*m\n', ...
    recomputedPeakInput);
fprintf('  俯仰1%%带内稳定时间，固件K=%.3f s，重算K=%.3f s\n', ...
    firmwarePitchMetrics.settlingTime1PercentS, ...
    recomputedPitchMetrics.settlingTime1PercentS);
fprintf('  固件K反向俯仰峰值: %.3f deg\n', ...
    rad2deg(firmwarePitchMetrics.oppositePeakRad));
fprintf('  CAD参考姿态19 kg支撑前馈: %.3f N/腿\n', ...
    configuration.axialFeedforwardN);
fprintf('  2 deg扰动时DM8009额定利用率: [%.1f%%, %.1f%%]\n', ...
    100 * cadControlCommand.rawFeasibility.jointRatedUtilization);

result.legLengths = legLengths;
result.totalMasses = totalMasses;
result.firmwareMaxRealPart = firmwareMaxRealPart;
result.recomputedMaxRealPart = recomputedMaxRealPart;
result.gainRelativeDifference = gainRelativeDifference;
result.nominalLength = nominalLength;
result.nominalModel = nominalModel;
result.firmwareSchedule = firmwareSchedule;
result.firmwareController = firmwareController;
result.recomputedController = recomputedController;
result.firmwareDisturbance = firmwareDisturbance;
result.recomputedDisturbance = recomputedDisturbance;
result.firmwarePeakInput = firmwarePeakInput;
result.recomputedPeakInput = recomputedPeakInput;
result.firmwarePitchMetrics = firmwarePitchMetrics;
result.recomputedPitchMetrics = recomputedPitchMetrics;
result.cadPose = cadPose;
result.cadControlCommand = cadControlCommand;
result.figurePath = "";
result.warning = [
    "固件K在暂估模型稳定不等于实机符号已确认"
    "当前比较同时包含参数、Q/R和坐标实现差异，不能只按元素接近度判断优劣"
    ];

if makeFigure
    figureHandle = figure('Name', 'Firmware gain comparison', ...
        'Color', 'white', 'Position', [100, 100, 1240, 780]);
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    bar([firmwareSchedule.K(:), recomputedController.K(:)]);
    grid on; xlabel('K元素（按列展开）'); ylabel('增益');
    title('L0=0.30 m的K数值比较'); legend('同构固件K', '19 kg重算K');

    nexttile;
    plot(legLengths, firmwareMaxRealPart, 'LineWidth', 1.4);
    hold on; yline(0, 'k--'); grid on;
    xlabel('腿长 L0 / m'); ylabel('最大闭环极点实部');
    title('固件K(L0)控制暂估模型');
    legend(compose('%g kg', totalMasses), 'Location', 'best');

    nexttile;
    plot(firmwareDisturbance.nonlinearTime, ...
        rad2deg(firmwareDisturbance.nonlinearState(:, 5)), 'LineWidth', 1.5);
    hold on;
    plot(recomputedDisturbance.nonlinearTime, ...
        rad2deg(recomputedDisturbance.nonlinearState(:, 5)), '--', 'LineWidth', 1.5);
    grid on; xlabel('时间 / s'); ylabel('机体俯仰角 / deg');
    title('名义模型2 deg非线性扰动'); legend('同构固件K', '重算K');

    nexttile;
    plot(legLengths, 100 * gainRelativeDifference, 'LineWidth', 1.4);
    grid on; xlabel('腿长 L0 / m'); ylabel('Frobenius相对差异 / %');
    title('固件K与各质量点重算K的整体差异');
    legend(compose('%g kg', totalMasses), 'Location', 'best');

    resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
    if ~isfolder(resultDir), mkdir(resultDir); end
    result.figurePath = fullfile(resultDir, '20_firmware_gain_comparison.png');
    exportgraphics(figureHandle, result.figurePath, 'Resolution', 160);
    fprintf('固件K比较图已保存: %s\n', result.figurePath);
end
end

function metrics = decayMetrics(time, signal)
initialMagnitude = abs(signal(1));
tolerance = 0.01 * initialMagnitude;
lastOutside = find(abs(signal) > tolerance, 1, 'last');
if isempty(lastOutside)
    settlingTime = 0;
elseif lastOutside == numel(time)
    settlingTime = inf;
else
    settlingTime = time(lastOutside + 1);
end

initialSign = sign(signal(1));
oppositeSignal = -initialSign * signal;
metrics.settlingTime1PercentS = settlingTime;
metrics.oppositePeakRad = max([0; oppositeSignal]);
metrics.initialMagnitudeRad = initialMagnitude;
metrics.toleranceRad = tolerance;
end
