function result = run_team_mass_sensitivity_6dof(makeFigure)
%RUN_TEAM_MASS_SENSITIVITY_6DOF Test one nominal K on 18-20 kg plants.

arguments
    makeFigure (1, 1) logical = true
end

totalMasses = [18; 19; 20];
nominalIndex = 2;
legLength = team_firmware_parameters().legLengthDefault;
nominalParameters = team_estimated_dynamics_parameters( ...
    totalMasses(nominalIndex), legLength);
nominalModel = linearize_equivalent_leg_6dof(nominalParameters);
nominalController = design_heu_lqr_6dof(nominalModel);

models = cell(numel(totalMasses), 1);
redesignedControllers = cell(numel(totalMasses), 1);
disturbances = cell(numel(totalMasses), 1);
nominalGainClosedLoopMaxRealPart = zeros(size(totalMasses));
redesignedGainClosedLoopMaxRealPart = zeros(size(totalMasses));
gainRelativeDifference = zeros(size(totalMasses));
peakInputNm = zeros(numel(totalMasses), 2);
for index = 1:numel(totalMasses)
    parameters = team_estimated_dynamics_parameters( ...
        totalMasses(index), legLength);
    models{index} = linearize_equivalent_leg_6dof(parameters);
    redesignedControllers{index} = design_heu_lqr_6dof(models{index});
    nominalGainClosedLoopMaxRealPart(index) = max(real(eig( ...
        models{index}.A - models{index}.B * nominalController.K)));
    redesignedGainClosedLoopMaxRealPart(index) = max(real( ...
        redesignedControllers{index}.closedLoopPoles));
    gainRelativeDifference(index) = norm( ...
        redesignedControllers{index}.K - nominalController.K, 'fro') / ...
        norm(nominalController.K, 'fro');
    disturbances{index} = simulate_initial_disturbance_6dof( ...
        models{index}, nominalController);
    peakInputNm(index, :) = max(abs(disturbances{index}.nonlinearInput), [], 1);
end

fprintf('本队六维暂估名义点: 总质量 %.1f kg, L0=%.3f m\n', ...
    totalMasses(nominalIndex), legLength);
for index = 1:numel(totalMasses)
    fprintf(['  %.0f kg: 名义K最大闭环极点实部 %.5f, ' ...
        '重算K相对变化 %.2f%%, 峰值[T,Tp]=[%.3f, %.3f] N*m\n'], ...
        totalMasses(index), nominalGainClosedLoopMaxRealPart(index), ...
        100 * gainRelativeDifference(index), peakInputNm(index, :));
end

result.totalMasses = totalMasses;
result.legLength = legLength;
result.nominalIndex = nominalIndex;
result.nominalParameters = nominalParameters;
result.nominalModel = nominalModel;
result.nominalController = nominalController;
result.models = models;
result.redesignedControllers = redesignedControllers;
result.disturbances = disturbances;
result.nominalGainClosedLoopMaxRealPart = nominalGainClosedLoopMaxRealPart;
result.redesignedGainClosedLoopMaxRealPart = redesignedGainClosedLoopMaxRealPart;
result.gainRelativeDifference = gainRelativeDifference;
result.peakInputNm = peakInputNm;
result.warning = "18~20 kg均为暂估模型；稳定只证明这一参数族内的数学鲁棒性";
result.figurePath = "";

if makeFigure
    figureHandle = figure('Name', 'Team mass sensitivity', ...
        'Color', 'white', 'Position', [120, 120, 1200, 760]);
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    hold on;
    for index = 1:numel(totalMasses)
        plot(disturbances{index}.nonlinearTime, ...
            rad2deg(disturbances{index}.nonlinearState(:, 5)), ...
            'LineWidth', 1.4, 'DisplayName', sprintf('%.0f kg', totalMasses(index)));
    end
    grid on; xlabel('时间 / s'); ylabel('机体俯仰角 / deg');
    title('同一19 kg名义K下的2 deg扰动'); legend('Location', 'best');

    nexttile;
    bar(totalMasses, nominalGainClosedLoopMaxRealPart);
    yline(0, 'k--'); grid on;
    xlabel('整车暂估总质量 / kg'); ylabel('最大闭环极点实部');
    title('冻结名义K的稳定性');

    nexttile;
    bar(totalMasses, 100 * gainRelativeDifference);
    grid on; xlabel('整车暂估总质量 / kg'); ylabel('K的Frobenius相对变化 / %');
    title('每个质量点重算K相对名义值的变化');

    nexttile;
    bar(totalMasses, peakInputNm);
    grid on; xlabel('整车暂估总质量 / kg'); ylabel('峰值控制量 / N m');
    title('非线性闭环峰值输入'); legend('T', 'T_p');

    resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
    if ~isfolder(resultDir), mkdir(resultDir); end
    result.figurePath = fullfile(resultDir, '19_team_mass_sensitivity.png');
    exportgraphics(figureHandle, result.figurePath, 'Resolution', 160);
    fprintf('质量灵敏度图已保存: %s\n', result.figurePath);
end
end
