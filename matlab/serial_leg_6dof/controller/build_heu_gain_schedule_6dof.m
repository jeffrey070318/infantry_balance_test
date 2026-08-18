function schedule = build_heu_gain_schedule_6dof(legLengths)
%BUILD_HEU_GAIN_SCHEDULE_6DOF Recompute and fit K over the source range.

arguments
    legLengths (1, :) double {mustBeFinite, mustBePositive} = 0.08:0.01:0.32
end

gainSamples = zeros(numel(legLengths), 12);
maxClosedLoopRealPart = zeros(numel(legLengths), 1);
controllabilityRank = zeros(numel(legLengths), 1);

for index = 1:numel(legLengths)
    parameters = heu_reference_parameters(legLengths(index));
    model = linearize_equivalent_leg_6dof(parameters);
    controller = design_heu_lqr_6dof(model);
    gainSamples(index, :) = reshape(controller.K.', 1, []);
    maxClosedLoopRealPart(index) = max(real(controller.closedLoopPoles));
    controllabilityRank(index) = rank(ctrb(model.A, model.B));
end

coefficients = zeros(12, 4);
fittedGains = zeros(size(gainSamples));
for element = 1:12
    coefficients(element, :) = polyfit(legLengths, gainSamples(:, element).', 3);
    fittedGains(:, element) = polyval(coefficients(element, :), legLengths).';
end

schedule.legLengths = legLengths;
schedule.gainSamples = gainSamples;
schedule.coefficients = coefficients;
schedule.fittedGains = fittedGains;
schedule.fitError = fittedGains - gainSamples;
schedule.maxAbsErrorByElement = max(abs(schedule.fitError), [], 1);
schedule.maxRelativeErrorByElement = schedule.maxAbsErrorByElement ./ ...
    max(abs(gainSamples), [], 1);
schedule.maxClosedLoopRealPart = maxClosedLoopRealPart;
schedule.controllabilityRank = controllabilityRank;
schedule.validRange = [min(legLengths), max(legLengths)];
schedule.polynomialConvention = "polyval([a3 a2 a1 a0], L0)";
end
