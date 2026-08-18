function result = simulate_scheduled_leg_length_6dof( ...
    schedule, controllerMode, initialPitchRad, sampleTime, duration, ...
    targetTimes, targetLengths, initialLegLength, maxLegLengthRate, ...
    delaySamples, kinematicParameters, actuators, configuration)
%SIMULATE_SCHEDULED_LEG_LENGTH_6DOF Quasi-static LPV K(L0) simulation.

arguments
    schedule (1, 1) struct
    controllerMode (1, 1) string {mustBeMember(controllerMode, ...
        ["scheduled", "fixed"])}
    initialPitchRad (1, 1) double {mustBeFinite}
    sampleTime (1, 1) double {mustBeFinite, mustBePositive} = 0.001
    duration (1, 1) double {mustBeFinite, mustBePositive} = 4
    targetTimes (1, :) double {mustBeFinite, mustBeNonnegative} = [0, 1.5, 3]
    targetLengths (1, :) double {mustBeFinite, mustBePositive} = [0.15, 0.32, 0.30]
    initialLegLength (1, 1) double {mustBeFinite, mustBePositive} = 0.30
    maxLegLengthRate (1, 1) double {mustBeFinite, mustBePositive} = 0.20
    delaySamples (1, 1) double {mustBeFinite, mustBeNonnegative} = 1
    kinematicParameters (1, 1) struct = cad_serial_leg_parameters()
    actuators (1, 1) struct = serial_leg_actuator_parameters()
    configuration (1, 1) struct = struct()
end

configuration = normalizeConfiguration(configuration, schedule.totalMass);

if numel(targetTimes) ~= numel(targetLengths) || targetTimes(1) ~= 0 || ...
        any(diff(targetTimes) <= 0) || targetTimes(end) > duration
    error('sixdof:GainSchedule:InvalidLengthProfile', ...
        '目标时间必须从0开始、严格递增、落在仿真时长内并与目标腿长等长。');
end
if any(targetLengths < schedule.validRange(1) | ...
        targetLengths > schedule.validRange(2)) || ...
        initialLegLength < schedule.validRange(1) || ...
        initialLegLength > schedule.validRange(2)
    error('sixdof:GainSchedule:ProfileOutOfRange', ...
        '腿长轨迹超出K(L0)有效范围。');
end
if delaySamples ~= round(delaySamples)
    error('sixdof:GainSchedule:InvalidDelay', 'delaySamples必须为整数。');
end

time = (0:sampleTime:duration).';
state = nan(numel(time), 6);
filteredState = nan(numel(time), 6);
legLength = nan(numel(time), 1);
targetLength = nan(numel(time), 1);
gainNorm = nan(numel(time), 1);
rawInput = nan(numel(time), 2);
appliedInput = nan(numel(time), 2);
rawJointTorque = nan(numel(time), 2);
outputAngles = nan(numel(time), 2);
jacobianCondition = nan(numel(time), 1);
state(1, :) = [0, 0, 0, 0, initialPitchRad, 0];
legLength(1) = initialLegLength;

pitchKickByIndex = zeros(numel(time), 1);
for eventIndex = 1:numel(configuration.disturbanceTimes)
    sampleIndex = round(configuration.disturbanceTimes(eventIndex) / sampleTime) + 1;
    if sampleIndex < 1 || sampleIndex > numel(time)
        error('sixdof:GainSchedule:DisturbanceOutOfRange', ...
            '扰动时刻必须落在仿真时间范围内。');
    end
    pitchKickByIndex(sampleIndex) = pitchKickByIndex(sampleIndex) + ...
        configuration.disturbancePitchRad(eventIndex);
end

previousRng = rng;
restoreRng = onCleanup(@() rng(previousRng));
rng(configuration.seed, 'twister');
filtered = state(1, :).';

fixedController = evaluate_team_gain_schedule_6dof(schedule, initialLegLength);
previousOutputAngles = kinematicParameters.referenceOutputAngles;
commandQueue = zeros(2, delaySamples);
jointLimit = actuators.jointOutputRatedTorqueMagnitudeNm;
wheelLimit = actuators.wheelTransmission.continuousWheelTorqueIdealNmCandidate;
axialForceN = configuration.plantTotalMass * ...
    team_firmware_parameters().gravity / 2;
divergenceIndex = NaN;

for index = 1:numel(time)
    if pitchKickByIndex(index) ~= 0
        state(index, 5) = state(index, 5) + pitchKickByIndex(index);
    end
    profileIndex = find(targetTimes <= time(index), 1, 'last');
    targetLength(index) = targetLengths(profileIndex);
    currentLength = legLength(index);
    inverse = cad_serial_leg_inverse_task_pose( ...
        [currentLength; 0], previousOutputAngles, kinematicParameters);
    previousOutputAngles = inverse.outputAngles;
    outputAngles(index, :) = inverse.outputAngles.';
    [taskJacobian, diagnostics] = cad_serial_leg_task_jacobian( ...
        inverse.outputAngles, kinematicParameters);
    jacobianCondition(index) = diagnostics.conditionNumber;

    if controllerMode == "scheduled"
        controller = evaluate_team_gain_schedule_6dof(schedule, currentLength);
    else
        controller = fixedController;
    end
    gainNorm(index) = norm(controller.K, 'fro');
    currentState = state(index, :).';
    measuredState = currentState + configuration.noiseStd .* randn(6, 1);
    filtered = filtered + configuration.filterAlpha .* (measuredState - filtered);
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
    step = time(index + 1) - time(index);
    parameters = team_estimated_dynamics_parameters( ...
        configuration.plantTotalMass, currentLength);
    nextState = rk4Step(currentState, command, step, parameters);
    if any(~isfinite(nextState)) || ...
            any(abs(nextState([1, 5])) > deg2rad(60)) || ...
            any(abs(nextState([2, 4, 6])) > 100) || abs(nextState(3)) > 20
        divergenceIndex = index + 1;
        break;
    end
    state(index + 1, :) = nextState.';
    lengthError = targetLength(index) - currentLength;
    legLength(index + 1) = currentLength + ...
        sign(lengthError) * min(abs(lengthError), maxLegLengthRate * step);
end

validRows = all(isfinite(state), 2) & isfinite(legLength);
validTime = time(validRows);
validState = state(validRows, :);
validLength = legLength(validRows);
validRawInput = rawInput(validRows, :);
validRawJointTorque = rawJointTorque(validRows, :);
jointSaturated = any(abs(validRawJointTorque) > jointLimit.', 2);
wheelSaturated = abs(validRawInput(:, 1)) > wheelLimit;
finalState = validState(end, :);
isDiverged = ~isnan(divergenceIndex);
steadyRows = validTime >= max(0, validTime(end) - 1);
steadyPitchRmsDeg = sqrt(mean(rad2deg(validState(steadyRows, 5)).^2));
steadyLegAngleRmsDeg = sqrt(mean(rad2deg(validState(steadyRows, 1)).^2));
steadyPitchRateRmsDegPerS = sqrt(mean( ...
    rad2deg(validState(steadyRows, 6)).^2));
isConverged = ~isDiverged && steadyPitchRmsDeg < 0.2 && ...
    steadyLegAngleRmsDeg < 0.2 && steadyPitchRateRmsDegPerS < 2;

result.controllerMode = controllerMode;
result.time = time;
result.state = state;
result.filteredState = filteredState;
result.legLength = legLength;
result.targetLength = targetLength;
result.gainNorm = gainNorm;
result.rawInput = rawInput;
result.appliedInput = appliedInput;
result.rawJointTorque = rawJointTorque;
result.outputAngles = outputAngles;
result.jacobianCondition = jacobianCondition;
result.initialPitchRad = initialPitchRad;
result.sampleTime = sampleTime;
result.maxLegLengthRate = maxLegLengthRate;
result.delaySamples = delaySamples;
result.axialForceN = axialForceN;
result.configuration = configuration;
result.isDiverged = isDiverged;
result.isConverged = isConverged;
result.divergenceIndex = divergenceIndex;
result.validDuration = validTime(end);
result.finalState = finalState;
result.maxBodyPitchDeg = max(abs(rad2deg(validState(:, 5))));
result.maxActualLegLengthRate = max(abs(diff(validLength) ./ diff(validTime)));
result.maxRawInputNm = max(abs(validRawInput), [], 1);
result.maxRawJointTorqueNm = max(abs(validRawJointTorque), [], 1);
result.jointSaturationFraction = nnz(jointSaturated) / numel(jointSaturated);
result.wheelSaturationFraction = nnz(wheelSaturated) / numel(wheelSaturated);
result.steadyPitchRmsDeg = steadyPitchRmsDeg;
result.steadyLegAngleRmsDeg = steadyLegAngleRmsDeg;
result.steadyPitchRateRmsDegPerS = steadyPitchRateRmsDegPerS;
result.warning = [
    "逐拍更新冻结腿长模型，不含L0_dot和L0_ddot引起的动力学耦合"
    "腿长保持0 deg任务角，未模拟两腿高度差、地面冲击和离地"
    ];
end

function configuration = normalizeConfiguration(configuration, nominalMass)
defaults.plantTotalMass = nominalMass;
defaults.noiseStd = zeros(6, 1);
defaults.filterAlpha = ones(6, 1);
defaults.seed = 1;
defaults.disturbanceTimes = zeros(1, 0);
defaults.disturbancePitchRad = zeros(1, 0);
fields = fieldnames(defaults);
for index = 1:numel(fields)
    name = fields{index};
    if ~isfield(configuration, name)
        configuration.(name) = defaults.(name);
    end
end
if ~isequal(size(configuration.noiseStd), [6, 1]) || ...
        ~isequal(size(configuration.filterAlpha), [6, 1])
    error('sixdof:GainSchedule:InvalidFilterConfiguration', ...
        'noiseStd和filterAlpha必须为6x1向量。');
end
if any(configuration.filterAlpha <= 0 | configuration.filterAlpha > 1) || ...
        configuration.plantTotalMass <= 0
    error('sixdof:GainSchedule:InvalidRobustnessConfiguration', ...
        '滤波系数必须在(0,1]内且被控对象质量必须为正。');
end
if numel(configuration.disturbanceTimes) ~= ...
        numel(configuration.disturbancePitchRad)
    error('sixdof:GainSchedule:InvalidDisturbanceConfiguration', ...
        '扰动时刻和俯仰扰动量必须一一对应。');
end
configuration.disturbanceTimes = configuration.disturbanceTimes(:).';
configuration.disturbancePitchRad = configuration.disturbancePitchRad(:).';
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
