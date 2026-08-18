function result = run_serial_leg_6dof(makeFigures)
%RUN_SERIAL_LEG_6DOF Run both the dynamics and mechanism interface checks.

arguments
    makeFigures (1, 1) logical = true
end

startup_serial_leg_6dof();
result.dynamics = run_heu_reference_baseline_6dof(makeFigures);
result.mechanism = run_closed_chain_interface_check(makeFigures);
result.cadTopology = run_cad_topology_check(makeFigures);
result.cadKinematics = run_cad_forward_kinematics_check(makeFigures);
result.cadTaskSpace = run_cad_task_space_check(makeFigures);
result.actuators = run_actuator_contract_check();
result.controlChain = run_control_chain_interface_check();
result.firmwareLegLength = run_firmware_leg_length_check(makeFigures);
result.teamMassSensitivity = run_team_mass_sensitivity_6dof(makeFigures);
result.firmwareGainComparison = run_firmware_gain_comparison_6dof(makeFigures);
result.fixedLegRobustness = run_fixed_leg_controller_robustness_6dof(makeFigures);
result.sensorDelayFilter = run_sensor_delay_filter_robustness_6dof(makeFigures);
result.teamGainSchedule = run_team_gain_schedule_6dof(makeFigures);
result.teamGainScheduleRobustness = ...
    run_team_gain_schedule_robustness_6dof(makeFigures);
result.releaseAudit = run_sixdof_release_audit_6dof();
result.actuatorEnvelope = run_cad_actuator_envelope_check(makeFigures);
result.workspaceActuatorLimits = ...
    run_cad_workspace_actuator_limit_check(makeFigures);
result.kinematicBoundary = run_cad_kinematic_boundary_check(makeFigures);
end
