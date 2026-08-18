function tests = test_serial_leg_6dof
%TEST_SERIAL_LEG_6DOF Regression tests for the structured six-state project.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(rootDir));
testCase.TestData.rootDir = rootDir;
end

function teardownOnce(testCase)
rmpath(genpath(testCase.TestData.rootDir));
end

function testReferenceParametersMatchSource(testCase)
p = heu_reference_parameters(0.18);
verifyEqual(testCase, p.wheelRadius, 0.1, 'AbsTol', 1e-12);
verifyEqual(testCase, p.lowerComDistance, 0.09, 'AbsTol', 1e-12);
verifyEqual(testCase, p.upperComDistance, 0.09, 'AbsTol', 1e-12);
verifyEqual(testCase, p.bodyComOffset, 0.066, 'AbsTol', 1e-12);
verifyEqual(testCase, p.wheelMass, 0.95, 'AbsTol', 1e-12);
verifyEqual(testCase, p.legMass, 0.15, 'AbsTol', 1e-12);
verifyEqual(testCase, p.bodyMass, 9.65, 'AbsTol', 1e-12);
end

function testNonlinearEquilibriumAndResidual(testCase)
p = heu_reference_parameters(0.18);
[derivative, diagnostics] = equivalent_leg_dynamics_6dof( ...
    zeros(6, 1), zeros(2, 1), p);
verifyEqual(testCase, derivative, zeros(6, 1), 'AbsTol', 1e-12);
verifyLessThan(testCase, norm(diagnostics.residual, inf), 1e-12);
verifyGreaterThan(testCase, diagnostics.reciprocalCondition, 1e-6);
end

function testLinearModelContract(testCase)
model = linearize_equivalent_leg_6dof(heu_reference_parameters(0.18));
verifySize(testCase, model.A, [6, 6]);
verifySize(testCase, model.B, [6, 2]);
verifyEqual(testCase, model.A([1, 3, 5], :), ...
    [0, 1, 0, 0, 0, 0; 0, 0, 0, 1, 0, 0; 0, 0, 0, 0, 0, 1], ...
    'AbsTol', 1e-10);
verifyEqual(testCase, model.B([1, 3, 5], :), zeros(3, 2), 'AbsTol', 1e-10);
verifyEqual(testCase, rank(ctrb(model.A, model.B)), 6);
end

function testLqrStabilizesReferenceModel(testCase)
model = linearize_equivalent_leg_6dof(heu_reference_parameters(0.18));
controller = design_heu_lqr_6dof(model);
verifySize(testCase, controller.K, [2, 6]);
verifyLessThan(testCase, max(real(controller.closedLoopPoles)), 0);
end

function testNonlinearSmallDisturbanceConverges(testCase)
model = linearize_equivalent_leg_6dof(heu_reference_parameters(0.18));
controller = design_heu_lqr_6dof(model);
result = simulate_initial_disturbance_6dof(model, controller);
verifyLessThan(testCase, result.finalAttitudeRateNorm, 1e-3);
verifyLessThan(testCase, abs(result.finalPositionError), 1e-2);
verifyLessThan(testCase, result.maxStateDifference, 5e-3);
end

function testGainScheduleRangeIsControllableAndStable(testCase)
schedule = build_heu_gain_schedule_6dof();
verifyEqual(testCase, schedule.controllabilityRank, 6 * ones(25, 1));
verifyLessThan(testCase, max(schedule.maxClosedLoopRealPart), 0);
verifySize(testCase, schedule.coefficients, [12, 4]);
verifyLessThan(testCase, max(schedule.maxRelativeErrorByElement), 5e-3);
end

function testClosedChainResidualAndTaskAngle(testCase)
geometry = placeholder_serial_leg_geometry();
pose = closed_chain_forward_kinematics(deg2rad([40; 140]), geometry);
verifyLessThan(testCase, norm(pose.closureResidual, inf), 1e-12);
verifyEqual(testCase, pose.legAngle, 0, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, pose.legLength, 0.3);
end

function testClosedChainJacobianMatchesFiniteDifference(testCase)
geometry = placeholder_serial_leg_geometry();
q = deg2rad([35; 132]);
J = closed_chain_task_jacobian(q, geometry);
h = 1e-7;
Jfd = zeros(2);
for column = 1:2
    delta = zeros(2, 1); delta(column) = h;
    plus = closed_chain_forward_kinematics(q + delta, geometry);
    minus = closed_chain_forward_kinematics(q - delta, geometry);
    Jfd(:, column) = ([plus.legLength; plus.legAngle] - ...
        [minus.legLength; minus.legAngle]) / (2 * h);
end
verifyEqual(testCase, J, Jfd, 'AbsTol', 1e-7);
end

function testVmcPreservesVirtualPower(testCase)
geometry = placeholder_serial_leg_geometry();
q = deg2rad([38; 137]);
taskForce = [140; -7];
qDot = [0.8; -0.3];
result = task_force_to_joint_torque(taskForce, q, geometry);
taskVelocity = result.jacobian * qDot;
verifyEqual(testCase, result.jointTorque.' * qDot, ...
    taskForce.' * taskVelocity, 'AbsTol', 1e-12);
end

function testUnreachableGeometryIsRejected(testCase)
geometry = placeholder_serial_leg_geometry();
geometry.passiveLink1 = 0.01;
geometry.passiveLink2 = 0.01;
verifyError(testCase, @() closed_chain_forward_kinematics( ...
    deg2rad([20; 160]), geometry), 'sixdof:Kinematics:Unreachable');
end

function testCadJointTopologyLoadsAndIsPlanar(testCase)
geometry = load_cad_serial_leg_geometry();
verifySize(testCase, geometry.pointsMm, [3, 7]);
verifyEmpty(testCase, geometry.missingRootTransforms);
verifyLessThan(testCase, range(geometry.pointsMm(1, :)), 1e-6);
verifyEqual(testCase, geometry.axisDirections, ...
    repmat([1; 0; 0], 1, 7), 'AbsTol', 1e-10);
end

function testCadParallelogramCandidateLengths(testCase)
geometry = load_cad_serial_leg_geometry();
links = geometry.linkAxisDistances;
lengths = [links.distance_mm];
verifyEqual(testCase, sum(abs(lengths - 55.81) < 2e-3), 2);
verifyEqual(testCase, sum(abs(lengths - 106.98) < 2e-3), 2);
verifyEqual(testCase, links(2).distance_mm, 109.9, 'AbsTol', 2e-3);
end

function testCadTwoOutputForwardKinematicsReproducesAssembly(testCase)
parameters = cad_serial_leg_parameters();
pose = cad_serial_leg_forward_kinematics( ...
    parameters.referenceOutputAngles, parameters);
verifyEqual(testCase, pose.nodes, parameters.referenceNodes, 'AbsTol', 2e-6);
verifyLessThan(testCase, max(abs(pose.closureResidual)), 1e-10);
end

function testCadTwoOutputKinematicsRespondsToBothInputs(testCase)
parameters = cad_serial_leg_parameters();
J = cad_serial_leg_node_jacobian( ...
    parameters.referenceOutputAngles, 7, parameters);
verifyEqual(testCase, size(J), [2, 2]);
verifyEqual(testCase, rank(J), 2);
verifyLessThan(testCase, cond(J), 100);
end

function testCadTwoOutputRejectsUnreachableLoop(testCase)
parameters = cad_serial_leg_parameters();
verifyError(testCase, @() cad_serial_leg_forward_kinematics( ...
    [0; 0], parameters), 'sixdof:CadKinematics:Unreachable');
end

function testCadWheelAttachmentEvidenceLoads(testCase)
geometry = load_cad_serial_leg_geometry();
verifyEqual(testCase, geometry.wheelRadiusM, 0.06, 'AbsTol', 1e-12);
verifyEqual(testCase, string(geometry.rigidAttachmentMatches(1).structural_component), ...
    "连杆4改A");
verifyGreaterThanOrEqual(testCase, ...
    geometry.rigidAttachmentMatches(1).coincident_axis_count, 7);
end

function testCadWheelCenterReproducesAssembly(testCase)
parameters = cad_serial_leg_parameters();
pose = cad_serial_leg_forward_kinematics( ...
    parameters.referenceOutputAngles, parameters);
verifyEqual(testCase, pose.wheelCenter, ...
    parameters.wheelCenterReference, 'AbsTol', 2e-6);
verifyGreaterThan(testCase, pose.legLength, 0.25);
verifyLessThan(testCase, pose.legLength, 0.40);
end

function testCadTaskJacobianIsFullRank(testCase)
parameters = cad_serial_leg_parameters();
[J, diagnostics] = cad_serial_leg_task_jacobian( ...
    parameters.referenceOutputAngles, parameters);
verifySize(testCase, J, [2, 2]);
verifyEqual(testCase, rank(J), 2);
verifyLessThan(testCase, diagnostics.conditionNumber, 100);
end

function testCadTaskVmcPreservesVirtualPower(testCase)
parameters = cad_serial_leg_parameters();
q = parameters.referenceOutputAngles;
taskForce = [130; -6];
qDot = [0.5; -0.8];
result = cad_task_force_to_output_torque(taskForce, q, parameters);
verifyEqual(testCase, result.outputTorque.' * qDot, ...
    taskForce.' * result.jacobian * qDot, 'AbsTol', 1e-12);
end

function testDmJ8009ManualParameters(testCase)
motor = dm_j8009_parameters();
verifyEqual(testCase, motor.ratedOutputTorqueNm, 20);
verifyEqual(testCase, motor.peakOutputTorqueNm, 40);
verifyEqual(testCase, motor.internalReductionRatio, 9);
verifyEqual(testCase, motor.ratedOutputSpeedRpm(1), 100);
verifyTrue(testCase, motor.mitTorqueRangeConfigurable);
end

function testM3508C620ManualParameters(testCase)
motor = m3508_c620_parameters();
verifyEqual(testCase, motor.internalReductionRatio, 3591 / 187, ...
    'AbsTol', 1e-12);
verifyEqual(testCase, motor.ratedOutputTorqueNm, 3);
verifyEqual(testCase, motor.stallOutputTorqueNm, 4.5);
verifyLessThan(testCase, motor.manualStallInputCurrentA, ...
    motor.ratedInputCurrentA);
end

function testJointSprocketRatioAndWheelGearBoundary(testCase)
actuators = serial_leg_actuator_parameters();
verifyEqual(testCase, actuators.jointTransmission.activeTeeth, [20; 20]);
verifyEqual(testCase, actuators.jointTransmission.drivenTeeth, [20; 20]);
verifyEqual(testCase, ...
    actuators.jointTransmission.outputPerMotorMagnitude, [1; 1]);
verifyFalse(testCase, actuators.jointTransmission.isCalibrated);
verifyTrue(testCase, ...
    actuators.wheelTransmission.selectedForSixDofSimulation);
verifyEqual(testCase, actuators.wheelTransmission.selectedTopology, ...
    "保持架固定、齿圈输出");
verifyEqual(testCase, actuators.wheelTransmission.externalGearRatio, ...
    -(2680 / 170), 'AbsTol', 1e-12);
verifyEqual(testCase, actuators.wheelTransmission.totalMotorToWheelRatio, ...
    -(3591 / 187) * (2680 / 170), 'AbsTol', 1e-10);
verifyFalse(testCase, actuators.wheelTransmission.isResolved);
verifyEqual(testCase, ...
    actuators.wheelTransmission.cadExternalRatioMagnitudeCandidates(1), ...
    2680 / 170, 'AbsTol', 1e-12);
verifyEqual(testCase, ...
    actuators.wheelTransmission.cadExternalRatioMagnitudeCandidates(2), ...
    1 / abs((-170 / 2680) / (-170 / 2680 - 1)), 'AbsTol', 1e-12);
verifyEqual(testCase, ...
    actuators.wheelTransmission.cascadedRatioMagnitudeCandidate, ...
    (3591 / 187) * (2680 / 170), 'AbsTol', 1e-10);
verifyEqual(testCase, ...
    actuators.wheelTransmission.ratedWheelSpeedRpmCandidate, ...
    469 / (2680 / 170), 'AbsTol', 1e-10);
end

function testUncalibratedJointAngleMappingIsRejected(testCase)
actuators = serial_leg_actuator_parameters();
verifyError(testCase, @() joint_motor_to_output_angles( ...
    [0; 0], actuators.jointTransmission), ...
    'sixdof:Actuator:CalibrationRequired');
end

function testCalibratedJointAngleMapping(testCase)
actuators = serial_leg_actuator_parameters();
transmission = actuators.jointTransmission;
transmission.rotationSign = [1; -1];
transmission.motorZeroRad = [0.2; -0.1];
transmission.outputZeroRad = [1.0; 2.0];
transmission.isCalibrated = true;
outputAngles = joint_motor_to_output_angles([0.5; 0.3], transmission);
verifyEqual(testCase, outputAngles, [1.3; 1.6], 'AbsTol', 1e-12);
end

function testActuatorFeasibilityAtZeroCommand(testCase)
parameters = cad_serial_leg_parameters();
result = evaluate_serial_leg_actuator_limits( ...
    [0; 0], parameters.referenceOutputAngles, 0, parameters);
verifyEqual(testCase, result.jointOutputTorqueNm, [0; 0], ...
    'AbsTol', 1e-12);
verifyTrue(testCase, result.withinAllRatedIdeal);
verifyTrue(testCase, result.withinAllPeakIdeal);
end

function testActuatorFeasibilityRejectsExcessiveCommands(testCase)
parameters = cad_serial_leg_parameters();
result = evaluate_serial_leg_actuator_limits( ...
    [5000; 500], parameters.referenceOutputAngles, 100, parameters);
verifyFalse(testCase, result.withinJointPeak);
verifyFalse(testCase, result.withinWheelPeakIdeal);
verifyFalse(testCase, result.withinAllPeakIdeal);
end

function testWheelIdealTorqueLimitsUseExternalStageOnly(testCase)
actuators = serial_leg_actuator_parameters();
externalRatio = 2680 / 170;
verifyEqual(testCase, ...
    actuators.wheelTransmission.continuousWheelTorqueIdealNmCandidate, ...
    3 * externalRatio, 'AbsTol', 1e-12);
verifyEqual(testCase, ...
    actuators.wheelTransmission.peakWheelTorqueIdealNmCandidate, ...
    4.5 * externalRatio, 'AbsTol', 1e-12);
end

function testWorkspaceActuatorLimitScanIncludesReferencePose(testCase)
result = run_cad_workspace_actuator_limit_check(false);
center = (numel(result.offsetDegrees) + 1) / 2;
verifyTrue(testCase, result.reachable(center, center));
verifyEqual(testCase, result.axialRatedLimitN(center, center), ...
    151.703, 'AbsTol', 2e-3);
verifyEqual(testCase, result.legTorqueRatedLimitNm(center, center), ...
    40.0, 'AbsTol', 2e-3);
verifyGreaterThan(testCase, result.minimumAxialRatedLimitN, 0);
verifyGreaterThan(testCase, result.minimumLegTorqueRatedLimitNm, 0);
end

function testExtendedKinematicBoundaryIncludesReferencePose(testCase)
result = run_cad_kinematic_boundary_check(false, 45, 31);
center = (numel(result.offsetDegrees) + 1) / 2;
verifyTrue(testCase, result.reachable(center, center));
verifyTrue(testCase, result.wellConditioned(center, center));
verifyLessThan(testCase, result.maximumClosureResidualM, 1e-9);
verifyEqual(testCase, result.legLengthM(center, center), ...
    0.306803, 'AbsTol', 2e-6);
verifyGreaterThan(testCase, result.maximumLegLengthM, ...
    result.legLengthM(center, center));
end

function testControlStepMatchesLqrPdAndVmcLayers(testCase)
kinematics = cad_serial_leg_parameters();
model = linearize_equivalent_leg_6dof(heu_reference_parameters(0.18));
controller = design_heu_lqr_6dof(model);
state = [0.01; -0.02; 0.03; -0.04; 0.02; -0.01];
stateReference = zeros(6, 1);
legState = [0.30; -0.02];
legReference = [0.31; 0];
configuration = struct('axialKpNPerM', 500, ...
    'axialKdNsPerM', 20, 'axialFeedforwardN', 80, ...
    'limitMode', "none");

command = serial_leg_control_step_6dof( ...
    state, stateReference, legState, legReference, ...
    kinematics.referenceOutputAngles, controller, configuration, kinematics);
expectedInput = -controller.K * state;
expectedF0 = 80 + 500 * 0.01 + 20 * 0.02;
expectedVmc = cad_task_force_to_output_torque( ...
    [expectedF0; expectedInput(2)], ...
    kinematics.referenceOutputAngles, kinematics);

verifyEqual(testCase, command.generalizedInputRaw, expectedInput, ...
    'AbsTol', 1e-12);
verifyEqual(testCase, command.axialForceRawN, expectedF0, 'AbsTol', 1e-12);
verifyEqual(testCase, command.jointOutputTorqueRawNm, ...
    expectedVmc.outputTorque, 'AbsTol', 1e-12);
verifyFalse(testCase, command.wasAnyLimited);
end

function testControlStepAppliesActuatorLimits(testCase)
kinematics = cad_serial_leg_parameters();
controller.K = zeros(2, 6);
controller.K(1, 1) = 1e5;
controller.K(2, 5) = 1e5;
configuration = struct('axialKpNPerM', 0, ...
    'axialKdNsPerM', 0, 'axialFeedforwardN', 5000, ...
    'limitMode', "rated");
actuators = serial_leg_actuator_parameters();

command = serial_leg_control_step_6dof( ...
    ones(6, 1), zeros(6, 1), [0.3; 0], [0.3; 0], ...
    kinematics.referenceOutputAngles, controller, configuration, ...
    kinematics, actuators);

verifyTrue(testCase, command.wasJointLimited);
verifyTrue(testCase, command.wasWheelLimited);
verifyLessThanOrEqual(testCase, abs(command.jointOutputTorqueLimitedNm), ...
    actuators.jointOutputRatedTorqueMagnitudeNm);
verifyLessThanOrEqual(testCase, abs(command.wheelTorqueLimitedNm), ...
    actuators.wheelTransmission.continuousWheelTorqueIdealNmCandidate);
end

function testFirmwareLegLengthContractMatchesCadReference(testCase)
firmware = team_firmware_parameters();
cad = cad_serial_leg_parameters();
pose = cad_serial_leg_forward_kinematics( ...
    cad.referenceOutputAngles, cad);

verifyEqual(testCase, firmware.wheelRadius, cad.wheelRadius, 'AbsTol', 1e-12);
verifyEqual(testCase, firmware.legLengthDefault, 0.30, 'AbsTol', 1e-12);
verifyEqual(testCase, firmware.legLengthRange, [0.15; 0.32], 'AbsTol', 1e-12);
verifyGreaterThanOrEqual(testCase, pose.legLength, firmware.legLengthRange(1));
verifyLessThanOrEqual(testCase, pose.legLength, firmware.legLengthRange(2));
end

function testEstimatedTeamMassClosesExactly(testCase)
parameters = team_estimated_dynamics_parameters(19, 0.30);
verifyEqual(testCase, parameters.totalMass, 19, 'AbsTol', 1e-12);
verifyEqual(testCase, parameters.massClosureResidual, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, parameters.bodyMass + ...
    2 * (parameters.legMass + parameters.wheelMass), 19, 'AbsTol', 1e-12);
verifyTrue(testCase, parameters.isProvisional);
end

function testNominalGainStabilizesMassEstimateRange(testCase)
result = run_team_mass_sensitivity_6dof(false);
verifyEqual(testCase, result.totalMasses, [18; 19; 20]);
verifyLessThan(testCase, result.nominalGainClosedLoopMaxRealPart, 0);
verifyLessThan(testCase, max(result.gainRelativeDifference), 0.1);
end

function testFirmwareGainMatchesActivePolynomialAtNominalLength(testCase)
schedule = firmware_gain_schedule_6dof(0.30);
expected = [
    -14.011382557900761, -2.267469007922622, -4.325296005156829, ...
    -4.777818153219379, 3.926416986825455, 0.316684891823535
      5.665983845771823,  1.032992841025540,  2.313663397643425, ...
     2.453283904192583, 30.584461366550165, 1.119426531398811
    ];
verifyEqual(testCase, schedule.K, expected, 'AbsTol', 1e-12);
verifyEqual(testCase, schedule.stateOrder, ...
    ["theta", "theta_dot", "x", "x_dot", "phi", "phi_dot"]);
end

function testFirmwareGainStabilizesEstimatedOperatingRange(testCase)
result = run_firmware_gain_comparison_6dof(false);
verifyLessThan(testCase, result.firmwareMaxRealPart, 0);
verifyLessThan(testCase, max(real(eig(result.nominalModel.A + ...
    result.nominalModel.B * result.firmwareSchedule.K))), 100);
verifyGreaterThan(testCase, max(real(eig(result.nominalModel.A + ...
    result.nominalModel.B * result.firmwareSchedule.K))), 0);
verifyTrue(testCase, result.cadControlCommand.rawFeasibility.withinAllRatedIdeal);
verifyGreaterThan(testCase, result.firmwarePitchMetrics.oppositePeakRad, 0);
verifyLessThan(testCase, result.recomputedPitchMetrics.settlingTime1PercentS, ...
    result.firmwarePitchMetrics.settlingTime1PercentS);
end

function testSampledSaturatedSmallDisturbanceConverges(testCase)
parameters = team_estimated_dynamics_parameters(19, 0.30);
model = linearize_equivalent_leg_6dof(parameters);
controller = design_heu_lqr_6dof(model);
cad = cad_serial_leg_parameters();
pose = cad_serial_leg_forward_kinematics(cad.referenceOutputAngles, cad);
result = simulate_sampled_saturated_disturbance_6dof( ...
    model, controller, deg2rad(2), 0.001, 5, ...
    cad.referenceOutputAngles, 19 * parameters.gravity / 2);
verifyTrue(testCase, result.isConverged);
verifyFalse(testCase, result.isDiverged);
verifyTrue(testCase, result.isLinearDiscreteStable);
verifyLessThan(testCase, pose.legLength, 0.32);
end

function testFixedLegRobustnessGridHasStableBaseline(testCase)
result = run_fixed_leg_controller_robustness_6dof(false);
verifyTrue(testCase, all(result.converged(1, 1, :)));
verifyFalse(testCase, any(result.diverged(1, 1, :)));
verifyLessThan(testCase, result.spectralRadius(1, :), 1);
verifyEqual(testCase, result.pose.legLength, 0.30, 'AbsTol', 1e-9);
verifyEqual(testCase, result.pose.legAngle, 0, 'AbsTol', 1e-9);
end

function testCadInverseTaskPoseFindsFirmwareDefault(testCase)
cad = cad_serial_leg_parameters();
result = cad_serial_leg_inverse_task_pose( ...
    [0.30; 0], cad.referenceOutputAngles, cad);
verifyEqual(testCase, result.pose.legLength, 0.30, 'AbsTol', 1e-10);
verifyEqual(testCase, result.pose.legAngle, 0, 'AbsTol', 1e-10);
verifyLessThan(testCase, norm(result.residual, inf), 1e-10);
end

function testNoisyDelayedBaselineConverges(testCase)
parameters = team_estimated_dynamics_parameters(19, 0.30);
model = linearize_equivalent_leg_6dof(parameters);
controller = design_heu_lqr_6dof(model);
cad = cad_serial_leg_parameters();
inverse = cad_serial_leg_inverse_task_pose( ...
    [0.30; 0], cad.referenceOutputAngles, cad);
configuration.noiseStd = [deg2rad(0.08); deg2rad(0.5); 0.002; 0.02; ...
    deg2rad(0.05); deg2rad(0.3)];
configuration.filterAlpha = [1; 1; 1; 0.5; 1; 1];
configuration.delaySamples = 1;
configuration.seed = 42;
simulation = simulate_noisy_delayed_disturbance_6dof( ...
    model, controller, deg2rad(10), 0.001, 4, inverse.outputAngles, ...
    19 * parameters.gravity / 2, configuration, cad);
verifyTrue(testCase, simulation.isConverged);
verifyFalse(testCase, simulation.isDiverged);
verifyLessThan(testCase, simulation.steadyPitchRmsDeg, 0.2);
end

function testSensorDelayFilterMonteCarloBaseline(testCase)
result = run_sensor_delay_filter_robustness_6dof(false, 2);
verifyEqual(testCase, result.convergenceRate, ones(4, 2));
verifyLessThan(testCase, result.meanPitchRmsDeg, 0.2);
end

function testTeamGainScheduleFitIsStable(testCase)
schedule = build_team_gain_schedule_6dof();
verifyEqual(testCase, schedule.controllabilityRank, 6 * ones(18, 1));
verifyLessThan(testCase, schedule.exactMaxRealPart, 0);
verifyLessThan(testCase, schedule.fittedMaxRealPart, 0);
verifyLessThan(testCase, max(schedule.maxRelativeErrorByElement), 1e-3);
end

function testTeamGainScheduleRejectsOutOfRangeLength(testCase)
schedule = build_team_gain_schedule_6dof();
verifyError(testCase, @() evaluate_team_gain_schedule_6dof(schedule, 0.14), ...
    'sixdof:GainSchedule:LengthOutOfRange');
end

function testRateLimitedGainScheduleSimulationConverges(testCase)
result = run_team_gain_schedule_6dof(false);
verifyTrue(testCase, result.simulations{1}.isConverged);
verifyFalse(testCase, result.simulations{1}.isDiverged);
verifyLessThanOrEqual(testCase, ...
    result.simulations{1}.maxActualLegLengthRate, 0.2 + 1e-10);
verifyEqual(testCase, min(result.simulations{1}.legLength), 0.15, ...
    'AbsTol', 1e-10);
verifyEqual(testCase, max(result.simulations{1}.legLength), 0.32, ...
    'AbsTol', 1e-10);
end


function testGainScheduleRobustnessGridConverges(testCase)
result = run_team_gain_schedule_robustness_6dof(false, 1);
verifyEqual(testCase, result.convergenceRate, ones(3, 2));
verifyLessThan(testCase, result.meanPitchRmsDeg, 0.2);
verifyLessThan(testCase, result.meanMaximumJointTorqueNm, 20);
end

function testSixDofReleaseAuditSeparatesMathFromHardware(testCase)
result = run_sixdof_release_audit_6dof();
verifyTrue(testCase, result.matlabChecksPass);
verifyFalse(testCase, result.readyForHardware);
verifyFalse(testCase, result.jointCalibrationComplete);
verifyFalse(testCase, result.wheelTopologyResolved);
end

function testEmptyCalibrationRecordCannotBeApplied(testCase)
record = calibration_record_template_6dof();
validation = validate_calibration_record_6dof(record);
verifyFalse(testCase, validation.ready);
verifyError(testCase, @() apply_calibration_record_6dof(record), ...
    'sixdof:Calibration:RecordNotReady');
end

function testCompleteCalibrationRecordCanBeApplied(testCase)
record = calibration_record_template_6dof();
record.timestamp = "2026-08-18T12:00:00+08:00";
record.operator = "test";
record.firmwareCommit = "test-commit";
record.batteryVoltageV = 24;
record.joint.rotationSign = [1; -1];
record.joint.motorZeroRad = [0.2; -0.1];
record.joint.outputZeroRad = deg2rad([38.174; 141.823]);
record.joint.outputErrorRad = deg2rad([0.2; -0.3]);
record.joint.isCalibrated = true;
record.imu.rightPitchSign = 1;
record.imu.leftPitchSign = -1;
record.imu.rightGyroYSign = 1;
record.imu.leftGyroYSign = -1;
record.imu.staticGyroRmsRadPerS = 0.01;
record.imu.isCalibrated = true;
record.wheel.motorToWheelRatioMagnitude = 302.7;
record.wheel.directionSign = -1;
record.wheel.efficiency = 0.8;
record.wheel.isResolved = true;
validation = validate_calibration_record_6dof(record);
actuators = apply_calibration_record_6dof(record);
verifyTrue(testCase, validation.ready);
verifyTrue(testCase, actuators.jointTransmission.isCalibrated);
verifyTrue(testCase, actuators.wheelTransmission.isResolved);
verifyEqual(testCase, joint_motor_to_output_angles( ...
    record.joint.motorZeroRad, actuators.jointTransmission), ...
    record.joint.outputZeroRad, 'AbsTol', 1e-12);
end
