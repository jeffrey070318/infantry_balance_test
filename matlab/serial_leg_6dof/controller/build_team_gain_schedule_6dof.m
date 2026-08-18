function schedule = build_team_gain_schedule_6dof(totalMass, legLengths)
%BUILD_TEAM_GAIN_SCHEDULE_6DOF Fit the provisional team-model K(L0).

arguments
    totalMass (1, 1) double {mustBeFinite, mustBePositive} = 19
    legLengths (1, :) double {mustBeFinite, mustBePositive} = 0.15:0.01:0.32
end

if numel(unique(legLengths)) < 4 || any(diff(legLengths) <= 0)
    error('sixdof:GainSchedule:InvalidLengthGrid', ...
        'legLengths必须严格递增且至少包含4个点。');
end

gainSamples = zeros(numel(legLengths), 12);
controllabilityRank = zeros(numel(legLengths), 1);
exactMaxRealPart = zeros(numel(legLengths), 1);
models = cell(numel(legLengths), 1);

for index = 1:numel(legLengths)
    parameters = team_estimated_dynamics_parameters(totalMass, legLengths(index));
    models{index} = linearize_equivalent_leg_6dof(parameters);
    controller = design_heu_lqr_6dof(models{index});
    gainSamples(index, :) = reshape(controller.K.', 1, []);
    controllabilityRank(index) = rank(ctrb(models{index}.A, models{index}.B));
    exactMaxRealPart(index) = max(real(controller.closedLoopPoles));
end

coefficients = zeros(12, 4);
fittedGains = zeros(size(gainSamples));
fittedMaxRealPart = zeros(numel(legLengths), 1);
for element = 1:12
    coefficients(element, :) = polyfit( ...
        legLengths, gainSamples(:, element).', 3);
    fittedGains(:, element) = polyval( ...
        coefficients(element, :), legLengths).';
end
for index = 1:numel(legLengths)
    fittedK = reshape(fittedGains(index, :), 6, 2).';
    fittedMaxRealPart(index) = max(real(eig( ...
        models{index}.A - models{index}.B * fittedK)));
end

fitError = fittedGains - gainSamples;
maxAbsErrorByElement = max(abs(fitError), [], 1);
maxRelativeErrorByElement = maxAbsErrorByElement ./ ...
    max(abs(gainSamples), [], 1);

schedule.totalMass = totalMass;
schedule.legLengths = legLengths;
schedule.gainSamples = gainSamples;
schedule.coefficients = coefficients;
schedule.fittedGains = fittedGains;
schedule.fitError = fitError;
schedule.maxAbsErrorByElement = maxAbsErrorByElement;
schedule.maxRelativeErrorByElement = maxRelativeErrorByElement;
schedule.controllabilityRank = controllabilityRank;
schedule.exactMaxRealPart = exactMaxRealPart;
schedule.fittedMaxRealPart = fittedMaxRealPart;
schedule.validRange = [legLengths(1), legLengths(end)];
schedule.stateOrder = ["theta", "theta_dot", "x", "x_dot", "phi", "phi_dot"];
schedule.outputOrder = ["wheel_torque_T", "leg_angle_torque_Tp"];
schedule.polynomialConvention = "polyval([a3 a2 a1 a0], L0)";
schedule.warning = [
    "由本队19 kg暂估参数族生成，不可直接下发实机"
    "逐腿长冻结线性化只构成LPV近似，不含伸缩速度和加速度耦合"
    ];
end
