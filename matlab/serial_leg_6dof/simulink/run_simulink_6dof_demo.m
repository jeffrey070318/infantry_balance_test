function result = run_simulink_6dof_demo(openModel)
%RUN_SIMULINK_6DOF_DEMO Build, simulate, plot, and optionally open the model.

arguments
    openModel (1, 1) logical = true
end

startup_serial_leg_6dof();
modelPath = build_serial_leg_6dof_model();
[~, modelName] = fileparts(modelPath);
load_system(modelPath);
simulationOutput = sim(modelName, 'ReturnWorkspaceOutputs', 'on');
pitch = simulationOutput.pitch_deg;
input = simulationOutput.input_T_Tp;

figureHandle = figure('Name', 'Simulink six-state demo', ...
    'Color', 'white', 'Position', [150, 150, 1100, 700]);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;
plot(pitch.Time, pitch.Data, 'LineWidth', 1.5);
grid on; xlabel('时间 / s'); ylabel('机体俯仰角 / deg');
title('Simulink六维LQR：10 deg初始扰动');
nexttile;
plot(input.Time, input.Data, 'LineWidth', 1.3);
grid on; xlabel('时间 / s'); ylabel('控制量 / N m');
title('LQR输出'); legend('轮矩T', '腿俯仰力矩T_p', 'Location', 'best');

rootDir = fileparts(fileparts(mfilename('fullpath')));
figurePath = fullfile(rootDir, 'results', '26_simulink_6dof_response.png');
exportgraphics(figureHandle, figurePath, 'Resolution', 160);
fprintf('Simulink模型: %s\n', modelPath);
fprintf('Simulink结果图: %s\n', figurePath);

result.modelPath = modelPath;
result.figurePath = figurePath;
result.pitch = pitch;
result.input = input;
result.finalPitchDeg = pitch.Data(end);
if openModel
    open_system(modelName);
    open_system(modelName + "/Pitch Scope");
else
    close_system(modelName, 0);
end
end
