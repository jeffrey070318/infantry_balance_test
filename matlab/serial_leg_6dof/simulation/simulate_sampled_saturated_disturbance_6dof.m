function result = simulate_sampled_saturated_disturbance_6dof( ...
    model, controller, initialPitchRad, sampleTime, duration, ...
    outputAngles, axialForceN, kinematicParameters, actuators)
%SIMULATE_SAMPLED_SATURATED_DISTURBANCE_6DOF Nonlinear sampled-data check.

arguments
    model (1, 1) struct
    controller (1, 1) struct
    initialPitchRad (1, 1) double {mustBeFinite}
    sampleTime (1, 1) double {mustBeFinite, mustBePositive}
    duration (1, 1) double {mustBeFinite, mustBePositive}
    outputAngles (2, 1) double {mustBeFinite}
    axialForceN (1, 1) double {mustBeFinite}
    kinematicParameters (1, 1) struct = cad_serial_leg_parameters()
    actuators (1, 1) struct = serial_leg_actuator_parameters()
end

time = (0:sampleTime:duration).';
if time(end) < duration
    time(end + 1, 1) = duration;
end
state = nan(numel(time), 6);
rawInput = nan(numel(time), 2);
appliedInput = nan(numel(time), 2);
rawJointTorque = nan(numel(time), 2);
appliedJointTorque = nan(numel(time), 2);
state(1, :) = [0, 0, 0, 0, initialPitchRad, 0];

[taskJacobian, jacobianDiagnostics] = cad_serial_leg_task_jacobian( ...
    outputAngles, kinematicParameters);
jointLimit = actuators.jointOutputRatedTorqueMagnitudeNm;
wheelLimit = actuators.wheelTransmission.continuousWheelTorqueIdealNmCandidate;
divergenceIndex = NaN;

for index = 1:numel(time)
    currentState = state(index, :).';
    rawInput(index, :) = (-controller.K * currentState).';
    rawJointTorque(index, :) = (taskJacobian.' * ...
        [axialForceN; rawInput(index, 2)]).';
    appliedJointTorque(index, :) = clampSymmetric( ...
        rawJointTorque(index, :).', jointLimit).';
    achievedTaskForce = taskJacobian.' \ appliedJointTorque(index, :).';
    appliedInput(index, :) = [clampSymmetric(rawInput(index, 1), wheelLimit), ...
        achievedTaskForce(2)];

    if index == numel(time)
        break;
    end
    step = time(index + 1) - time(index);
    nextState = rk4Step(currentState, appliedInput(index, :).', step, ...
        model.parameters);
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
validRawInput = rawInput(validRows, :);
validAppliedInput = appliedInput(validRows, :);
validRawJointTorque = rawJointTorque(validRows, :);
validAppliedJointTorque = appliedJointTorque(validRows, :);
jointSaturated = any(abs(validRawJointTorque) > jointLimit.', 2);
wheelSaturated = abs(validRawInput(:, 1)) > wheelLimit;

finalState = validState(end, :);
isDiverged = ~isnan(divergenceIndex);
isConverged = ~isDiverged && ...
    all(abs(finalState([1, 5])) < deg2rad(0.1)) && ...
    all(abs(finalState([2, 6])) < deg2rad(1)) && ...
    abs(finalState(4)) < 0.02;

discretePlant = c2d(ss(model.A, model.B, eye(6), zeros(6, 2)), ...
    sampleTime, 'zoh');
spectralRadius = max(abs(eig( ...
    discretePlant.A - discretePlant.B * controller.K)));

result.time = time;
result.state = state;
result.rawInput = rawInput;
result.appliedInput = appliedInput;
result.rawJointTorque = rawJointTorque;
result.appliedJointTorque = appliedJointTorque;
result.sampleTime = sampleTime;
result.initialPitchRad = initialPitchRad;
result.axialForceN = axialForceN;
result.outputAngles = outputAngles;
result.taskJacobian = taskJacobian;
result.jacobianDiagnostics = jacobianDiagnostics;
result.jointLimitNm = jointLimit;
result.wheelLimitNm = wheelLimit;
result.spectralRadius = spectralRadius;
result.isLinearDiscreteStable = spectralRadius < 1;
result.divergenceIndex = divergenceIndex;
result.isDiverged = isDiverged;
result.isConverged = isConverged;
result.validDuration = validTime(end);
result.maxBodyPitchDeg = max(abs(rad2deg(validState(:, 5))));
result.maxLegAngleDeg = max(abs(rad2deg(validState(:, 1))));
result.maxRawInputNm = max(abs(validRawInput), [], 1);
result.maxAppliedInputNm = max(abs(validAppliedInput), [], 1);
result.maxRawJointTorqueNm = max(abs(validRawJointTorque), [], 1);
result.maxAppliedJointTorqueNm = max(abs(validAppliedJointTorque), [], 1);
result.jointSaturationFraction = nnz(jointSaturated) / numel(jointSaturated);
result.wheelSaturationFraction = nnz(wheelSaturated) / numel(wheelSaturated);
result.finalState = finalState;
result.warning = "固定腿长、固定CAD姿态、无传感器噪声和通信延迟";
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
