function result = run_sixdof_release_audit_6dof()
%RUN_SIXDOF_RELEASE_AUDIT_6DOF Check the MATLAB/firmware interface contract.

firmware = team_firmware_parameters();
firmwareSchedule = firmware_gain_schedule_6dof( ...
    firmware.legLengthDefault);
teamSchedule = build_team_gain_schedule_6dof();
actuators = serial_leg_actuator_parameters();

result.stateOrderMatches = isequal(firmwareSchedule.stateOrder, ...
    ["theta", "theta_dot", "x", "x_dot", "phi", "phi_dot"]);
result.outputOrderMatches = isequal(firmwareSchedule.outputOrder, ...
    ["wheel_torque_T", "leg_angle_torque_Tp"]);
result.legLengthContractMatches = ...
    firmware.legLengthDefault == 0.30 && ...
    isequal(firmware.legLengthRange, [0.15; 0.32]) && ...
    firmware.maxLegLengthRate == 0.20;
result.firmwareSourceMatches = contains(firmwareSchedule.source, ...
    "balance_infantry/Robot/Controller/kinematics.c");
result.signConventionRecorded = contains( ...
    firmwareSchedule.matlabAuditedExpression, "u=-K*x");
result.teamScheduleFitStable = ...
    max(teamSchedule.maxRelativeErrorByElement) < 1e-3 && ...
    all(teamSchedule.fittedMaxRealPart < 0);
result.jointCalibrationComplete = actuators.jointTransmission.isCalibrated;
result.wheelTopologyResolved = actuators.wheelTransmission.isResolved;
result.matlabChecksPass = all([
    result.stateOrderMatches
    result.outputOrderMatches
    result.legLengthContractMatches
    result.firmwareSourceMatches
    result.signConventionRecorded
    result.teamScheduleFitStable
    ]);
result.readyForHardware = result.matlabChecksPass && ...
    result.jointCalibrationComplete && result.wheelTopologyResolved;
result.blockers = [
    "DM8009零位、方向和效率尚未标定"
    "轮边拓扑效率和轮编码器正方向尚未确认"
    "IMU安装变换尚未完成实机标定"
    "18~20 kg质量与惯量仍为第一版暂估"
    ];

fprintf('六维封板审计:\n');
fprintf('  MATLAB接口合同: %s\n', passText(result.matlabChecksPass));
fprintf('  实机下发就绪: %s\n', passText(result.readyForHardware));
fprintf('  DM8009标定完成: %s\n', passText(result.jointCalibrationComplete));
fprintf('  轮侧拓扑已解析: %s\n', passText(result.wheelTopologyResolved));
end

function text = passText(value)
if value
    text = 'PASS';
else
    text = 'BLOCKED';
end
end
