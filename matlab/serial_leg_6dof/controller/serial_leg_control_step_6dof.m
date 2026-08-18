function command = serial_leg_control_step_6dof( ...
    state, stateReference, legState, legReference, outputAngles, ...
    controller, configuration, kinematicParameters, actuators)
%SERIAL_LEG_CONTROL_STEP_6DOF Compose LQR, leg-length PD, VMC and limits.

arguments
    state (6, 1) double {mustBeFinite}
    stateReference (6, 1) double {mustBeFinite}
    legState (2, 1) double {mustBeFinite}
    legReference (2, 1) double {mustBeFinite}
    outputAngles (2, 1) double {mustBeFinite}
    controller (1, 1) struct
    configuration (1, 1) struct = defaultConfiguration()
    kinematicParameters (1, 1) struct = cad_serial_leg_parameters()
    actuators (1, 1) struct = serial_leg_actuator_parameters()
end

requiredControllerFields = "K";
requiredConfigurationFields = ["axialKpNPerM", "axialKdNsPerM", ...
    "axialFeedforwardN", "limitMode"];
requireFields(controller, requiredControllerFields, "controller");
requireFields(configuration, requiredConfigurationFields, "configuration");

stateError = state - stateReference;
generalizedInputRaw = -controller.K * stateError;
wheelTorqueRaw = generalizedInputRaw(1);
legAngleTorqueRaw = generalizedInputRaw(2);

lengthError = legReference(1) - legState(1);
lengthRateError = legReference(2) - legState(2);
axialForceRaw = configuration.axialFeedforwardN + ...
    configuration.axialKpNPerM * lengthError + ...
    configuration.axialKdNsPerM * lengthRateError;
taskForceRaw = [axialForceRaw; legAngleTorqueRaw];

vmcRaw = cad_task_force_to_output_torque( ...
    taskForceRaw, outputAngles, kinematicParameters);
[jointLimit, wheelLimit] = selectLimits(configuration.limitMode, actuators);
jointTorqueLimited = clampSymmetric(vmcRaw.outputTorque, jointLimit);
wheelTorqueLimited = clampSymmetric(wheelTorqueRaw, wheelLimit);

% Independent joint saturation changes both equivalent task-space channels.
taskForceAchieved = vmcRaw.jacobian.' \ jointTorqueLimited;
rawFeasibility = evaluate_serial_leg_actuator_limits( ...
    taskForceRaw, outputAngles, wheelTorqueRaw, ...
    kinematicParameters, actuators);

command.stateError = stateError;
command.lengthError = lengthError;
command.lengthRateError = lengthRateError;
command.generalizedInputRaw = generalizedInputRaw;
command.wheelTorqueRawNm = wheelTorqueRaw;
command.legAngleTorqueRawNm = legAngleTorqueRaw;
command.axialForceRawN = axialForceRaw;
command.taskForceRaw = taskForceRaw;
command.jointOutputTorqueRawNm = vmcRaw.outputTorque;
command.wheelTorqueLimitedNm = wheelTorqueLimited;
command.jointOutputTorqueLimitedNm = jointTorqueLimited;
command.taskForceAchievedAfterJointLimit = taskForceAchieved;
command.jointTorqueLimitNm = jointLimit;
command.wheelTorqueLimitNm = wheelLimit;
command.wasJointLimited = any(abs(jointTorqueLimited - vmcRaw.outputTorque) > 1e-12);
command.wasWheelLimited = abs(wheelTorqueLimited - wheelTorqueRaw) > 1e-12;
command.wasAnyLimited = command.wasJointLimited || command.wasWheelLimited;
command.vmcRaw = vmcRaw;
command.rawFeasibility = rawFeasibility;
command.configuration = configuration;
command.contract = [
    "LQR输出=[轮端转矩T; 等效腿角力矩Tp]"
    "腿长PD输出轴向力F0，VMC输入=[F0; Tp]"
    "输出为机构两个主动轴转矩和轮端转矩，尚不是电机电流命令"
    ];
command.warning = [
    "configuration中的前馈和PD增益必须按本队质量与试验重新给定"
    "关节零位、正方向、效率和轮侧实机转矩标定未完成，禁止直接下发"
    ];
end

function configuration = defaultConfiguration()
configuration.axialKpNPerM = 0;
configuration.axialKdNsPerM = 0;
configuration.axialFeedforwardN = 0;
configuration.limitMode = "rated";
end

function requireFields(value, fields, label)
for index = 1:numel(fields)
    if ~isfield(value, fields(index))
        error('sixdof:Controller:MissingField', ...
            '%s缺少字段%s。', label, fields(index));
    end
end
end

function [jointLimit, wheelLimit] = selectLimits(mode, actuators)
mode = string(mode);
switch mode
    case "rated"
        jointLimit = actuators.jointOutputRatedTorqueMagnitudeNm;
        wheelLimit = actuators.wheelTransmission. ...
            continuousWheelTorqueIdealNmCandidate;
    case "peak"
        jointLimit = actuators.jointOutputPeakTorqueMagnitudeNm;
        wheelLimit = actuators.wheelTransmission.peakWheelTorqueIdealNmCandidate;
    case "none"
        jointLimit = inf(2, 1);
        wheelLimit = inf;
    otherwise
        error('sixdof:Controller:InvalidLimitMode', ...
            'limitMode必须是rated、peak或none。');
end
end

function limited = clampSymmetric(value, magnitudeLimit)
limited = min(max(value, -magnitudeLimit), magnitudeLimit);
end
