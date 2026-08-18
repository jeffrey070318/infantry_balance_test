function tests = test_single_6dof
%TEST_SINGLE_6DOF Regression tests for the RM reference model.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
tutorialDir = fileparts(fileparts(mfilename('fullpath')));
addpath(tutorialDir);
testCase.TestData.tutorialDir = tutorialDir;
end

function teardownOnce(testCase)
rmpath(testCase.TestData.tutorialDir);
end

function testModelDimensions(testCase)
model = paper_model_6dof();

verifySize(testCase, model.A, [6 6]);
verifySize(testCase, model.B, [6 2]);
verifySize(testCase, model.C, [6 6]);
verifySize(testCase, model.D, [6 2]);
end

function testStateAndInputContracts(testCase)
model = paper_model_6dof();

expectedStates = ["theta", "theta_dot", "x_b", "x_b_dot", ...
    "phi", "phi_dot"];
expectedStateUnits = ["rad", "rad/s", "m", "m/s", "rad", "rad/s"];
expectedInputs = ["T", "T_p"];

verifyEqual(testCase, model.stateNames, expectedStates);
verifyEqual(testCase, model.stateUnits, expectedStateUnits);
verifyEqual(testCase, model.inputNames, expectedInputs);
verifyEqual(testCase, model.inputUnits, ["N*m", "N*m"]);
verifyEqual(testCase, model.legLength, 0.18, 'AbsTol', 1e-12);
end

function testPublishedMatrixCoefficients(testCase)
model = paper_model_6dof();

expectedA = [
      0,       1, 0, 0,        0, 0;
    265.9556,  0, 0, 0,  80.6327, 0;
      0,       0, 0, 1,        0, 0;
    -25.4562,  0, 0, 0,   1.8637, 0;
      0,       0, 0, 0,        0, 1;
    156.6952,  0, 0, 0, 183.0614, 0];

expectedB = [
      0,       0;
    -15.1389, 13.8563;
      0,       0;
      2.1208, -0.7158;
      0,       0;
     -4.2238, 16.8001];

verifyEqual(testCase, model.A, expectedA, 'AbsTol', 1e-12);
verifyEqual(testCase, model.B, expectedB, 'AbsTol', 1e-12);
end

function testModelIsControllable(testCase)
model = paper_model_6dof();

verifyEqual(testCase, rank(ctrb(model.A, model.B)), 6);
end

function testPublishedLqrDesign(testCase)
model = paper_model_6dof();
controller = design_lqr_6dof(model);

expectedQ = diag([1, 1, 500, 100, 5000, 1]);
expectedR = diag([1, 0.25]);
expectedK = [
    -44.3788, -6.8496, -22.2828, -21.5569,  28.7706, 4.3751;
     11.2006,  0.7339,   3.7300,   3.2058, 151.7300, 4.6387];

verifyEqual(testCase, controller.Q, expectedQ);
verifyEqual(testCase, controller.R, expectedR);
% The RM reference prints A, B, and K to four decimal places. Recomputing K from
% the rounded A and B values differs from the printed K by at most 0.00151.
verifyEqual(testCase, controller.K, expectedK, 'AbsTol', 2e-3);
verifyTrue(testCase, all(real(controller.closedLoopPoles) < 0));
end

function testInitialPitchDisturbanceConverges(testCase)
model = paper_model_6dof();
controller = design_lqr_6dof(model);

t = linspace(0, 10, 2001).';
x0 = zeros(6, 1);
x0(5) = deg2rad(2);
closedLoop = ss(model.A - model.B * controller.K, zeros(6, 1), ...
    eye(6), zeros(6, 1));
x = initial(closedLoop, x0, t);

verifyLessThan(testCase, norm(x(end, :), inf), 1e-4);
end

function testTutorialRunnerReturnsCompleteSimulation(testCase)
result = run_single_6dof(false);

verifySize(testCase, result.x, [2001 6]);
verifySize(testCase, result.u, [2001 2]);
verifySize(testCase, result.K, [2 6]);
verifyEqual(testCase, result.x(1, 5), deg2rad(2), 'AbsTol', 1e-12);
verifyLessThan(testCase, norm(result.x(end, :), inf), 1e-4);
verifyTrue(testCase, all(real(result.closedLoopPoles) < 0));
end

function testSpeedReferenceMatchesRmScenario(testCase)
t = (0:0.005:8).';
reference = speed_reference_6dof(t, 1.5, 3.0);

verifySize(testCase, reference.xd, [1601 6]);
verifyEqual(testCase, reference.xd(:, 1:2), zeros(1601, 2));
verifyEqual(testCase, reference.xd(:, 5:6), zeros(1601, 2));
verifyEqual(testCase, reference.xd(1, 4), 0, 'AbsTol', 1e-12);
verifyEqual(testCase, reference.xd(2, 4), 1.5, 'AbsTol', 1e-12);
verifyEqual(testCase, reference.xd(t >= 3, 4), zeros(nnz(t >= 3), 1));
verifyEqual(testCase, reference.xd(end, 3), 4.5, 'AbsTol', 1e-12);
end

function testSpeedTrackingRunnerReturnsCompleteSimulation(testCase)
result = run_speed_tracking_6dof(false);

verifySize(testCase, result.x, [1601 6]);
verifySize(testCase, result.xd, [1601 6]);
verifySize(testCase, result.u, [1601 2]);
verifyEqual(testCase, result.xd(end, 3), 4.5, 'AbsTol', 1e-12);
verifyLessThan(testCase, abs(result.x(end, 4)), 1e-3);
verifyLessThan(testCase, abs(result.x(end, 3) - 4.5), 1e-3);
verifyTrue(testCase, all(isfinite(result.x), 'all'));
verifyTrue(testCase, all(isfinite(result.u), 'all'));
verifyFalse(testCase, result.isWithinSmallAngleRange);
end

function testLimitedSpeedReferenceIsContinuousAndIntegrated(testCase)
t = (0:0.005:8).';
reference = limited_speed_reference_6dof(t, 1.5, 3.0, 1.0);

verifyEqual(testCase, reference.rampTime, 1.5, 'AbsTol', 1e-12);
verifyEqual(testCase, reference.stopTime, 4.5, 'AbsTol', 1e-12);
verifyEqual(testCase, reference.velocity(t == 1.5), 1.5, 'AbsTol', 1e-12);
verifyEqual(testCase, reference.velocity(t == 3.0), 1.5, 'AbsTol', 1e-12);
verifyEqual(testCase, reference.velocity(t == 4.5), 0, 'AbsTol', 1e-12);
verifyEqual(testCase, reference.position(end), 4.5, 'AbsTol', 1e-12);
verifyGreaterThanOrEqual(testCase, min(diff(reference.position)), -1e-12);
end

function testLimitedSpeedTrackingReducesCommandPeaks(testCase)
result = run_limited_speed_tracking_6dof(false);

verifySize(testCase, result.limited.x, [2001 6]);
verifySize(testCase, result.limited.u, [2001 2]);
verifyLessThan(testCase, result.limited.maxWheelTorque, ...
    result.ideal.maxWheelTorque);
verifyLessThan(testCase, result.limited.maxLegAngle, ...
    result.ideal.maxLegAngle);
verifyLessThan(testCase, abs(result.limited.x(end, 4)), 1e-3);
verifyLessThan(testCase, abs(result.limited.x(end, 3) - 4.5), 1e-3);
verifyTrue(testCase, result.limited.isWithinSmallAngleRange);
end

function testAccelerationSweepFindsProvisionalBoundary(testCase)
result = run_acceleration_sweep_6dof(false);

verifySize(testCase, result.table, [6 6]);
verifyEqual(testCase, result.recommendedAcceleration, 0.7, ...
    'AbsTol', 1e-12);
verifyTrue(testCase, result.table.Within15Deg(3));
verifyFalse(testCase, result.table.Within15Deg(4));
end

function testSampledUnlimitedMatchesContinuousResponse(testCase)
model = paper_model_6dof();
controller = design_lqr_6dof(model);
t = (0:0.001:10).';
reference = limited_speed_reference_6dof(t, 1.5, 3.0, 0.7);
continuous = simulate_tracking_6dof(model, controller, reference);
sampled = simulate_sampled_tracking_6dof( ...
    model, controller, reference, [Inf; Inf]);

verifyEqual(testCase, sampled.sampleTime, 0.001, 'AbsTol', 1e-12);
verifySize(testCase, sampled.x, [10001 6]);
verifyLessThan(testCase, max(abs(sampled.x - continuous.x), [], 'all'), ...
    2e-3);
verifyFalse(testCase, any(sampled.wasSaturated));
end

function testTorqueSaturationActuallyClipsCommands(testCase)
result = run_discrete_saturation_6dof(false);

verifyLessThanOrEqual(testCase, max(abs(result.insufficient.u(:, 1)), ...
    [], 'omitmissing'), ...
    2.5 + 1e-12);
verifyLessThanOrEqual(testCase, max(abs(result.insufficient.u(:, 2)), ...
    [], 'omitmissing'), ...
    1.0 + 1e-12);
verifyGreaterThan(testCase, result.insufficient.saturationFraction, 0);
verifyTrue(testCase, result.insufficient.isDiverged);
verifyLessThan(testCase, result.insufficient.divergenceTime, 10);
verifyFalse(testCase, result.adequate.isDiverged);
verifyLessThan(testCase, abs(result.adequate.finalPositionError), 1e-3);
end

function testSampleTimeSweepFindsDiscreteStabilityBoundary(testCase)
result = run_sample_time_sweep_6dof(false);

verifySize(testCase, result.table, [11 6]);
verifyTrue(testCase, result.table.EigenvalueStable(6));
verifyFalse(testCase, result.table.EigenvalueStable(end));
verifyGreaterThanOrEqual(testCase, result.largestStablePeriod, 0.020);
verifyLessThanOrEqual(testCase, result.firstUnstablePeriod, 0.030);
verifyLessThan(testCase, result.largestStablePeriod, ...
    result.firstUnstablePeriod);
end

function testOpenSourceGainScheduleMatchesItsSamples(testCase)
result = run_gain_schedule_method_6dof(false);

verifySize(testCase, result.samples.flatGains, [25 12]);
% The source publishes cubic coefficients to four decimal places.
verifyLessThan(testCase, max(result.maxAbsError), 5e-2);
verifyFalse(testCase, result.isCompatibleWithPaperModel);
verifyGreaterThan(testCase, ...
    max(abs(result.differenceAt018), [], 'all'), 10);
end

function testOpenSourceGainScheduleRejectsExtrapolation(testCase)
verifyError(testCase, @() opensource_gain_schedule_6dof(0.10), ...
    'sixdof:GainSchedule:OutOfRange');
verifyError(testCase, @() opensource_gain_schedule_6dof(0.40), ...
    'sixdof:GainSchedule:OutOfRange');
end

function testParameterizedOpenSourceModelMatchesGainSamples(testCase)
result = run_parameterized_model_validation_6dof(false);

verifyEqual(testCase, min(result.controllabilityRanks), 6);
verifyLessThan(testCase, max(result.maxClosedLoopRealPart), 0);
verifyLessThan(testCase, result.maxAbsGainError, 1e-3);
end

function testParameterizedModelChangesWithLegLength(testCase)
shortModel = opensource_parameterized_model_6dof(0.12);
longModel = opensource_parameterized_model_6dof(0.36);

verifyNotEqual(testCase, shortModel.A, longModel.A);
verifyNotEqual(testCase, shortModel.B, longModel.B);
verifyEqual(testCase, shortModel.stateNames, longModel.stateNames);
verifyEqual(testCase, shortModel.inputNames, longModel.inputNames);
end

function testFixedVersusScheduledGainSimulationIsFinite(testCase)
result = run_fixed_vs_scheduled_gain_6dof(false);

verifySize(testCase, result.xFixed, [2001 6]);
verifySize(testCase, result.xScheduled, [2001 6]);
verifyTrue(testCase, all(isfinite(result.xFixed), 'all'));
verifyTrue(testCase, all(isfinite(result.xScheduled), 'all'));
verifyLessThan(testCase, max(result.fixedMaxReal), 0);
verifyLessThan(testCase, max(result.scheduledMaxReal), 0);
verifyTrue(testCase, result.isFrozenParameterApproximation);
verifyLessThan(testCase, result.longLegThetaRms(2), ...
    result.longLegThetaRms(1));
verifyLessThan(testCase, result.longLegWheelTorqueRms(2), ...
    result.longLegWheelTorqueRms(1));
end
