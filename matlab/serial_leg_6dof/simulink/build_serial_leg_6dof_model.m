function modelPath = build_serial_leg_6dof_model()
%BUILD_SERIAL_LEG_6DOF_MODEL Create the minimal linear Simulink demo.

rootDir = fileparts(fileparts(mfilename('fullpath')));
modelName = 'serial_leg_6dof_demo';
modelPath = fullfile(fileparts(mfilename('fullpath')), modelName + ".slx");

parameters = team_estimated_dynamics_parameters(19, 0.30);
linearModel = linearize_equivalent_leg_6dof(parameters);
controller = design_heu_lqr_6dof(linearModel);
initialState = [0; 0; 0; 0; deg2rad(10); 0];

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if isfile(modelPath)
    delete(modelPath);
end
new_system(modelName);
modelWorkspace = get_param(modelName, 'ModelWorkspace');
assignin(modelWorkspace, 'A6', linearModel.A);
assignin(modelWorkspace, 'B6', linearModel.B);
assignin(modelWorkspace, 'C6', eye(6));
assignin(modelWorkspace, 'D6', zeros(6, 2));
assignin(modelWorkspace, 'K6', controller.K);
assignin(modelWorkspace, 'x0', initialState);

add_block('simulink/Continuous/State-Space', modelName + "/Six-state plant", ...
    'A', 'A6', 'B', 'B6', 'C', 'C6', 'D', 'D6', ...
    'X0', 'x0', 'Position', [300, 185, 455, 255]);
add_block('simulink/Math Operations/Gain', modelName + "/LQR -K", ...
    'Gain', '-K6', 'Multiplication', 'Matrix(K*u)', ...
    'Position', [565, 190, 690, 250]);
add_block('simulink/Signal Routing/Demux', modelName + "/State demux", ...
    'Outputs', '6', 'Position', [500, 45, 505, 155]);
add_block('simulink/Math Operations/Gain', modelName + "/rad to deg", ...
    'Gain', '180/pi', 'Position', [590, 95, 680, 135]);
add_block('simulink/Sinks/Scope', modelName + "/Pitch Scope", ...
    'Position', [790, 80, 850, 140]);
add_block('simulink/Sinks/To Workspace', modelName + "/Pitch to workspace", ...
    'VariableName', 'pitch_deg', 'SaveFormat', 'Timeseries', ...
    'Position', [745, 145, 875, 185]);
add_block('simulink/Sinks/To Workspace', modelName + "/State to workspace", ...
    'VariableName', 'state_6dof', 'SaveFormat', 'Timeseries', ...
    'Position', [500, 295, 630, 335]);
add_block('simulink/Sinks/To Workspace', modelName + "/Input to workspace", ...
    'VariableName', 'input_T_Tp', 'SaveFormat', 'Timeseries', ...
    'Position', [730, 220, 860, 260]);

add_line(modelName, 'Six-state plant/1', 'LQR -K/1', 'autorouting', 'on');
add_line(modelName, 'LQR -K/1', 'Six-state plant/1', 'autorouting', 'on');
add_line(modelName, 'Six-state plant/1', 'State demux/1', 'autorouting', 'on');
add_line(modelName, 'State demux/5', 'rad to deg/1', 'autorouting', 'on');
add_line(modelName, 'rad to deg/1', 'Pitch Scope/1', 'autorouting', 'on');
add_line(modelName, 'rad to deg/1', 'Pitch to workspace/1', 'autorouting', 'on');
add_line(modelName, 'Six-state plant/1', 'State to workspace/1', 'autorouting', 'on');
add_line(modelName, 'LQR -K/1', 'Input to workspace/1', 'autorouting', 'on');

set_param(modelName, 'StopTime', '4', 'Solver', 'ode4', ...
    'SolverType', 'Fixed-step', 'FixedStep', '0.001');
save_system(modelName, modelPath);

resultsDir = fullfile(rootDir, 'results');
if ~isfolder(resultsDir), mkdir(resultsDir); end
diagramPath = fullfile(resultsDir, '25_simulink_model.png');
print(['-s', modelName], '-dpng', '-r150', diagramPath);
close_system(modelName, 0);
end
