function result = simulate_noisy_delayed_disturbance_6dof( ...
    model, controller, initialPitchRad, sampleTime, duration, ...
    outputAngles, axialForceN, configuration, kinematicParameters, actuators)
%SIMULATE_NOISY_DELAYED_DISTURBANCE_6DOF Add measurement and execution effects.

arguments
    model (1, 1) struct
    controller (1, 1) struct
    initialPitchRad (1, 1) double {mustBeFinite}
    sampleTime (1, 1) double {mustBeFinite, mustBePositive}
    duration (1, 1) double {mustBeFinite, mustBePositive}
    outputAngles (2, 1) double {mustBeFinite}
    axialForceN (1, 1) double {mustBeFinite}
    configuration (1, 1) struct
    kinematicParameters (1, 1) struct = cad_serial_leg_parameters()
    actuators (1, 1) struct = serial_leg_actuator_parameters()
end

requiredFields = ["noiseStd", "filterAlpha", "delaySamples", "seed"];
for field = requiredFields
    if ~isfield(configuration, field)
        error('sixdof:Simulation:MissingConfiguration', ...
            '噪声/延迟配置缺少字段%s。', field);
    end
end
if ~isequal(size(configuration.noiseStd), [6, 1]) || ...
        ~isequal(size(configuration.filterAlpha), [6, 1])
    error('sixdof:Simulation:InvalidConfigurationSize', ...
        'noiseStd和filterAlpha必须为6x1向量。');
end
if any(configuration.filterAlpha <= 0 | configuration.filterAlpha > 1)
    error('sixdof:Simulation:InvalidFilterAlpha', ...
        'filterAlpha必须在(0,1]内。');
end

previousRng = rng;
restoreRng = onCleanup(@() rng(previousRng));
rng(configuration.seed, 'twister');

time = (0:sampleTime:duration).';
state = nan(numel(time), 6);
measuredState = nan(size(state));
filteredState = nan(size(state));
rawInput = nan(numel(time), 2);
appliedInput = nan(numel(time), 2);
rawJointTorque = nan(numel(time), 2);
state(1, :) = [0, 0, 0, 0, initialPitchRad, 0];
filtered = state(1, :).';

[taskJacobian, jacobianDiagnostics] = cad_serial_leg_task_jacobian( ...
    outputAngles, kinematicParameters);
jointLimit = actuators.jointOutputRatedTorqueMagnitudeNm;
wheelLimit = actuators.wheelTransmission.continuousWheelTorqueIdealNmCandidate;
delaySamples = round(configuration.delaySamples);
commandQueue = zeros(2, delaySamples);
divergenceIndex = NaN;

for index = 1:numel(time)
    currentState = state(index, :).';
    measured = currentState + configuration.noiseStd .* randn(6, 1);
    filtered = filtered + configuration.filterAlpha .* (measured - filtered);
    measuredState(index, :) = measured.';
    filteredState(index, :) = filtered.';

    rawCommand = -controller.K * filtered;
    rawInput(index, :) = rawCommand.';
    rawJoint = taskJacobian.' * [axialForceN; rawCommand(2)];
    rawJointTorque(index, :) = rawJoint.';
    limitedJoint = clampSymmetric(rawJoint, jointLimit);
    achievedTask = taskJacobian.' \ limitedJoint;
    candidateCommand = [clampSymmetric(rawCommand(1), wheelLimit); achievedTask(2)];

    if delaySamples == 0
        command = candidateCommand;
    else
        command = commandQueue(:, 1);
        commandQueue = [commandQueue(:, 2:end), candidateCommand];
    end
    appliedInput(index, :) = command.';

    if index == numel(time)
        break;
    end
    nextState = rk4Step(currentState, command, sampleTime, model.parameters);
    if any(~isfinite(nextState)) || ...
            any(abs(nextState([1, 5])) > deg2rad(60)) || ...
            any(abs(nextState([2, 4, 6])) > 100) || abs(nextState(3)) > 20
        divergenceIndex = index + 1;
        break;
    end
    state(index + 1, :) = nextState.';
end

validRows = all(isfinite(state), 2);
validTime = time(validRows);
validState = state(validRows, :);
validAppliedInput = appliedInput(validRows, :);
validRawJointTorque = rawJointTorque(validRows, :);
steadyRows = validTime >= max(0, validTime(end) - 1);
steadyState = validState(steadyRows, :);
steadyInput = validAppliedInput(steadyRows, :);
steadyJoint = validRawJointTorque(steadyRows, :);
jointSaturated = any(abs(validRawJointTorque) > jointLimit.', 2);

isDiverged = ~isnan(divergenceIndex);
steadyPitchRmsDeg = sqrt(mean(rad2deg(steadyState(:, 5)).^2));
steadyLegAngleRmsDeg = sqrt(mean(rad2deg(steadyState(:, 1)).^2));
isConverged = ~isDiverged && steadyPitchRmsDeg < 0.2 && ...
    steadyLegAngleRmsDeg < 0.2 && ...
    sqrt(mean(rad2deg(steadyState(:, 6)).^2)) < 2;

result.time = time;
result.state = state;
result.measuredState = measuredState;
result.filteredState = filteredState;
result.rawInput = rawInput;
result.appliedInput = appliedInput;
result.rawJointTorque = rawJointTorque;
result.configuration = configuration;
result.taskJacobian = taskJacobian;
result.jacobianDiagnostics = jacobianDiagnostics;
result.isDiverged = isDiverged;
result.divergenceIndex = divergenceIndex;
result.isConverged = isConverged;
result.validDuration = validTime(end);
result.steadyPitchRmsDeg = steadyPitchRmsDeg;
result.steadyLegAngleRmsDeg = steadyLegAngleRmsDeg;
result.steadyWheelTorqueRmsNm = sqrt(mean(steadyInput(:, 1).^2));
result.steadyLegTorqueRmsNm = sqrt(mean(steadyInput(:, 2).^2));
result.steadyJointTorqueStdNm = std(steadyJoint, 0, 1);
result.maxRawJointTorqueNm = max(abs(validRawJointTorque), [], 1);
result.jointSaturationFraction = nnz(jointSaturated) / numel(jointSaturated);
result.warning = "噪声标准差为工程假设，不是传感器实测标定";
end

function nextState = rk4Step(state, input, step, parameters)
k1 = equivalent_leg_dynamics_6dof(state, input, parameters);
k2 = equivalent_leg_dynamics_6dof(state + 0.5 * step * k1, input, parameters);
k3 = equivalent_leg_dynamics_6dof(state + 0.5 * step * k2, input, parameters);
k4 = equivalent_leg_dynamics_6dof(state + step * k3, input, parameters);
nextState = state + step * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
end

function limited = clampSymmetric(value, magnitudeLimit)
limited = min(max(value, -magnitudeLimit), magnitudeLimit);
end
