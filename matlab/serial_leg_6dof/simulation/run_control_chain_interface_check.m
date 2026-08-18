function result = run_control_chain_interface_check()
%RUN_CONTROL_CHAIN_INTERFACE_CHECK Exercise the open-source control layers.

kinematics = cad_serial_leg_parameters();
pose = cad_serial_leg_forward_kinematics( ...
    kinematics.referenceOutputAngles, kinematics);

% HEU parameters are retained only to exercise the software interface here.
model = linearize_equivalent_leg_6dof( ...
    heu_reference_parameters(pose.legLength));
controller = design_heu_lqr_6dof(model);
state = zeros(6, 1);
state(5) = deg2rad(2);
configuration.axialKpNPerM = 800;
configuration.axialKdNsPerM = 80;
configuration.axialFeedforwardN = 100;
configuration.limitMode = "rated";

command = serial_leg_control_step_6dof( ...
    state, zeros(6, 1), [pose.legLength; 0], ...
    [pose.legLength; 0], kinematics.referenceOutputAngles, ...
    controller, configuration, kinematics);

fprintf('控制链接口检查（仍使用哈工程动力学参数）:\n');
fprintf('  LQR原始输出 T=%.3f N*m, Tp=%.3f N*m\n', ...
    command.wheelTorqueRawNm, command.legAngleTorqueRawNm);
fprintf('  腿长通道 F0=%.3f N\n', command.axialForceRawN);
fprintf('  VMC主动轴原始转矩=[%.3f, %.3f] N*m\n', ...
    command.jointOutputTorqueRawNm);
fprintf('  额定限幅后主动轴转矩=[%.3f, %.3f] N*m, 轮矩=%.3f N*m\n', ...
    command.jointOutputTorqueLimitedNm, command.wheelTorqueLimitedNm);
fprintf('  注意: 本检查只证明控制层接口连通，不代表当前K可用于实机。\n');

result.pose = pose;
result.model = model;
result.controller = controller;
result.command = command;
result.warning = "哈工程参数仅用于接口检查，本队总成参数尚未替换";
end
