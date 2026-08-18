function result = run_team_gain_schedule_robustness_6dof(makeFigure, trialCount)
%RUN_TEAM_GAIN_SCHEDULE_ROBUSTNESS_6DOF Mass/noise/disturbance grid for K(L0).

arguments
    makeFigure (1, 1) logical = true
    trialCount (1, 1) double {mustBeInteger, mustBePositive} = 3
end

firmware = team_firmware_parameters();
schedule = build_team_gain_schedule_6dof(19, ...
    firmware.legLengthRange(1):0.01:firmware.legLengthRange(2));
plantMasses = [18; 19; 20];
modes = ["scheduled", "fixed"];
noiseStd = [deg2rad(0.08); deg2rad(0.5); 0.002; 0.02; ...
    deg2rad(0.05); deg2rad(0.3)];
shape = [numel(plantMasses), numel(modes), trialCount];
converged = false(shape);
steadyPitchRmsDeg = nan(shape);
maximumJointTorqueNm = nan(shape);
jointSaturationFraction = nan(shape);
representative = cell(numel(plantMasses), numel(modes));
cad = cad_serial_leg_parameters();
actuators = serial_leg_actuator_parameters();

for massIndex = 1:numel(plantMasses)
    for modeIndex = 1:numel(modes)
        for trialIndex = 1:trialCount
            configuration.plantTotalMass = plantMasses(massIndex);
            configuration.noiseStd = noiseStd;
            configuration.filterAlpha = [1; 1; 1; 0.5; 1; 1];
            configuration.seed = 1000 * massIndex + 100 * modeIndex + trialIndex;
            configuration.disturbanceTimes = [1.0, 2.6];
            configuration.disturbancePitchRad = deg2rad([8, 8]);
            simulation = simulate_scheduled_leg_length_6dof( ...
                schedule, modes(modeIndex), 0, 0.001, 4, ...
                [0, 1.5, 3], [0.15, 0.32, 0.30], ...
                firmware.legLengthDefault, firmware.maxLegLengthRate, 1, ...
                cad, actuators, configuration);
            converged(massIndex, modeIndex, trialIndex) = simulation.isConverged;
            steadyPitchRmsDeg(massIndex, modeIndex, trialIndex) = ...
                simulation.steadyPitchRmsDeg;
            maximumJointTorqueNm(massIndex, modeIndex, trialIndex) = ...
                max(simulation.maxRawJointTorqueNm);
            jointSaturationFraction(massIndex, modeIndex, trialIndex) = ...
                simulation.jointSaturationFraction;
            if trialIndex == 1
                representative{massIndex, modeIndex} = simulation;
            end
        end
    end
end

convergenceRate = mean(converged, 3);
meanPitchRmsDeg = mean(steadyPitchRmsDeg, 3);
meanMaximumJointTorqueNm = mean(maximumJointTorqueNm, 3);
meanJointSaturationFraction = mean(jointSaturationFraction, 3);

fprintf('变腿长K(L0)鲁棒性网格: %d次固定种子重复\n', trialCount);
for massIndex = 1:numel(plantMasses)
    for modeIndex = 1:numel(modes)
        fprintf(['  %.0f kg %s: 收敛率 %.1f%%, 稳态pitch RMS %.4f deg, ' ...
            '最大关节转矩 %.3f N*m, 饱和占比 %.3f%%\n'], ...
            plantMasses(massIndex), modes(modeIndex), ...
            100 * convergenceRate(massIndex, modeIndex), ...
            meanPitchRmsDeg(massIndex, modeIndex), ...
            meanMaximumJointTorqueNm(massIndex, modeIndex), ...
            100 * meanJointSaturationFraction(massIndex, modeIndex));
    end
end

result.schedule = schedule;
result.plantMasses = plantMasses;
result.modes = modes;
result.trialCount = trialCount;
result.converged = converged;
result.convergenceRate = convergenceRate;
result.meanPitchRmsDeg = meanPitchRmsDeg;
result.meanMaximumJointTorqueNm = meanMaximumJointTorqueNm;
result.meanJointSaturationFraction = meanJointSaturationFraction;
result.representative = representative;
result.disturbanceTimes = [1.0, 2.6];
result.disturbancePitchDeg = [8, 8];
result.warning = [
    "短腿和长腿扰动为仿真状态瞬时增加8 deg，不代表实体冲击能量"
    "仍为准静态六维LPV，不包含伸缩惯性、触地冲击和离地"
    ];
result.figurePath = "";

if makeFigure
    labels = ["本队K(L0)", "固定K"];
    figureHandle = figure('Name', 'Team gain schedule robustness', ...
        'Color', 'white', 'Position', [100, 100, 1400, 850]);
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    bar(plantMasses, 100 * convergenceRate);
    ylim([0, 105]); grid on; xlabel('被控对象总质量 / kg'); ylabel('收敛率 / %');
    title(sprintf('%d次噪声重复', trialCount)); legend(labels, 'Location', 'best');

    nexttile;
    hold on;
    for modeIndex = 1:numel(modes)
        simulation = representative{2, modeIndex};
        plot(simulation.time, rad2deg(simulation.state(:, 5)), 'LineWidth', 1.3);
    end
    xline(1.0, 'k:'); xline(2.6, 'k:'); grid on;
    xlabel('时间 / s'); ylabel('机体俯仰角 / deg');
    title('19 kg：短腿与长腿各8 deg状态扰动'); legend(labels, 'Location', 'best');

    nexttile;
    bar(plantMasses, meanPitchRmsDeg);
    grid on; xlabel('被控对象总质量 / kg'); ylabel('最后1 s俯仰RMS / deg');
    title('噪声下稳态姿态抖动'); legend(labels, 'Location', 'best');

    nexttile;
    bar(plantMasses, meanMaximumJointTorqueNm);
    yline(20, 'k--', 'DM8009额定'); grid on;
    xlabel('被控对象总质量 / kg'); ylabel('最大关节原始转矩 / N m');
    title('两次扰动中的关节转矩需求'); legend(labels, 'Location', 'best');

    resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
    if ~isfolder(resultDir), mkdir(resultDir); end
    result.figurePath = fullfile(resultDir, '24_team_gain_schedule_robustness.png');
    exportgraphics(figureHandle, result.figurePath, 'Resolution', 160);
    fprintf('变腿长鲁棒性图已保存: %s\n', result.figurePath);
end
end
