function result = simulate_sampled_tracking_6dof(model, controller, reference, torqueLimits)
%SIMULATE_SAMPLED_TRACKING_6DOF Run a sampled controller with saturation.
%
% At sample k:
%   u_raw[k] = K * (xd[k] - x[k])
%   u[k]     = clamp(u_raw[k], -limit, +limit)
%   x[k+1]   = Ad*x[k] + Bd*u[k]
%
% Ad and Bd are the exact zero-order-hold discretization of the continuous
% reference plant over one control period.

arguments
    model (1, 1) struct
    controller (1, 1) struct
    reference (1, 1) struct
    torqueLimits (2, 1) double {mustBePositive} = [Inf; Inf]
end

t = reference.t;
sampleTimes = diff(t);
assert(~isempty(sampleTimes), 'At least two time samples are required.');
sampleTime = sampleTimes(1);
assert(all(abs(sampleTimes - sampleTime) < 1e-12), ...
    'Reference time samples must have a fixed period.');

discretePlant = c2d(ss(model.A, model.B, eye(6), zeros(6, 2)), ...
    sampleTime, 'zoh');
Ad = discretePlant.A;
Bd = discretePlant.B;

n = numel(t);
x = zeros(n, 6);
uRaw = zeros(n, 2);
u = zeros(n, 2);
divergenceStateLimit = 100;
divergenceIndex = NaN;

for index = 1:n
    uRaw(index, :) = (controller.K * ...
        (reference.xd(index, :) - x(index, :)).').';
    u(index, :) = min(max(uRaw(index, :), -torqueLimits.'), ...
        torqueLimits.');

    if index < n
        nextState = (Ad * x(index, :).' + Bd * u(index, :).').';
        if any(~isfinite(nextState)) || any(abs(nextState) > divergenceStateLimit)
            divergenceIndex = index + 1;
            x(index + 1:end, :) = NaN;
            uRaw(index + 1:end, :) = NaN;
            u(index + 1:end, :) = NaN;
            break;
        end
        x(index + 1, :) = nextState;
    end
end

validRows = all(isfinite(x), 2);
validX = x(validRows, :);
validU = u(validRows, :);
validURaw = uRaw(validRows, :);

result.t = t;
result.sampleTime = sampleTime;
result.Ad = Ad;
result.Bd = Bd;
result.xd = reference.xd;
result.x = x;
result.uRaw = uRaw;
result.u = u;
result.torqueLimits = torqueLimits;
result.wasSaturated = any(abs(uRaw) > torqueLimits.', 2);
result.saturationFraction = nnz(result.wasSaturated(validRows)) / ...
    nnz(validRows);
result.divergenceStateLimit = divergenceStateLimit;
result.divergenceIndex = divergenceIndex;
result.isDiverged = ~isnan(divergenceIndex);
if result.isDiverged
    result.divergenceTime = t(divergenceIndex);
else
    result.divergenceTime = NaN;
end
result.maxLegAngle = max(abs(validX(:, 1)));
result.maxBodyPitch = max(abs(validX(:, 5)));
result.maxAppliedTorque = max(abs(validU), [], 1).';
result.maxRequestedTorque = max(abs(validURaw), [], 1).';
result.finalPositionError = reference.position(find(validRows, 1, 'last')) - ...
    validX(end, 3);
result.finalVelocityError = reference.velocity(find(validRows, 1, 'last')) - ...
    validX(end, 4);
result.smallAngleLimit = deg2rad(15);
result.isWithinSmallAngleRange = ...
    ~result.isDiverged && ...
    result.maxLegAngle <= result.smallAngleLimit && ...
    result.maxBodyPitch <= result.smallAngleLimit;
end
