function schedule = opensource_gain_schedule_6dof(legLength)
%OPENSOURCE_GAIN_SCHEDULE_6DOF Evaluate a separate RM open-source K(L0).
%
% Source:
%   建模和仿真开源/综合运动控制验证/6. 综合运动控制验证/K-L0/K.m
%
% This schedule uses different robot parameters and LQR weights from
% paper_model_6dof. It demonstrates the fitting method only and must not be
% used as the controller for that model or for the user's future robot.

arguments
    legLength (:, 1) double {mustBeFinite, mustBePositive}
end

validRange = [0.12, 0.36];
if any(legLength < validRange(1) | legLength > validRange(2))
    error('sixdof:GainSchedule:OutOfRange', ...
        'Leg length must stay inside the fitted range [0.12, 0.36] m.');
end

% Each row is [a3, a2, a1, a0] for a3*L0^3 + ... + a0.
% Row order: K11..K16, then K21..K26.
coefficients = [
   -172.5944,  194.4583, -102.6573, -3.2160;
     -0.9840,    0.4770,   -8.5684, -0.3764;
    -38.8812,   40.3986,  -15.0820, -2.3214;
    -25.9829,   28.2485,  -12.5490, -2.7215;
    -53.1626,   71.4193,  -37.8708,  9.8810;
     -8.7478,   11.6234,   -6.1338,  1.7402;
    182.7742, -134.1998,   17.2318, 11.5442;
      5.0007,   -1.7221,   -1.6219,  2.0283;
    -34.7106,   55.1695,  -32.5200,  8.4420;
    -55.6192,   72.6502,  -37.0784,  8.8727;
    214.7065, -218.2300,   79.9589,  4.8045;
     36.3354,  -37.3084,   13.9211,  0.5130];

flatGains = zeros(numel(legLength), 12);
for gainIndex = 1:12
    flatGains(:, gainIndex) = polyval(coefficients(gainIndex, :), ...
        legLength);
end

K = zeros(2, 6, numel(legLength));
for lengthIndex = 1:numel(legLength)
    K(1, :, lengthIndex) = flatGains(lengthIndex, 1:6);
    K(2, :, lengthIndex) = flatGains(lengthIndex, 7:12);
end
if isscalar(legLength)
    K = K(:, :, 1);
end

schedule.legLength = legLength;
schedule.K = K;
schedule.flatGains = flatGains;
schedule.coefficients = coefficients;
schedule.validRange = validRange;
schedule.sourceModelWeights = diag([1, 1, 20, 5, 50, 1]);
schedule.sourceInputWeights = diag([1, 0.25]);
schedule.warning = [ ...
    "Method demonstration from a different RM model; " ...
    "not compatible with paper_model_6dof."];
end
