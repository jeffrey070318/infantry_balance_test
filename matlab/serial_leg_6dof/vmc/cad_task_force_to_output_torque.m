function result = cad_task_force_to_output_torque(taskForce, outputAngles, parameters)
%CAD_TASK_FORCE_TO_OUTPUT_TORQUE Apply virtual work at the two J1 outputs.

arguments
    taskForce (2, 1) double {mustBeFinite}
    outputAngles (2, 1) double {mustBeFinite}
    parameters (1, 1) struct = cad_serial_leg_parameters()
end

[jacobian, diagnostics] = cad_serial_leg_task_jacobian( ...
    outputAngles, parameters);
result.outputTorque = jacobian.' * taskForce;
result.taskForce = taskForce;
result.jacobian = jacobian;
result.diagnostics = diagnostics;
result.contract = "taskForce=[轴向力N; 等效腿角力矩N*m]";
result.warning = "输出力矩位于J1两个机构输出轴，尚未换算到电机侧";
end
