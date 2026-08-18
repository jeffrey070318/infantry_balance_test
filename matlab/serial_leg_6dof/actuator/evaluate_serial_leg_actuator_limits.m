function result = evaluate_serial_leg_actuator_limits( ...
    taskForce, outputAngles, wheelTorqueNm, parameters, actuators)
%EVALUATE_SERIAL_LEG_ACTUATOR_LIMITS Check ideal output-side torque margins.

arguments
    taskForce (2, 1) double {mustBeFinite}
    outputAngles (2, 1) double {mustBeFinite}
    wheelTorqueNm (1, 1) double {mustBeFinite}
    parameters (1, 1) struct = cad_serial_leg_parameters()
    actuators (1, 1) struct = serial_leg_actuator_parameters()
end

vmc = cad_task_force_to_output_torque(taskForce, outputAngles, parameters);
jointTorque = vmc.outputTorque;
jointRatedLimit = actuators.jointOutputRatedTorqueMagnitudeNm;
jointPeakLimit = actuators.jointOutputPeakTorqueMagnitudeNm;
wheelContinuousLimit = ...
    actuators.wheelTransmission.continuousWheelTorqueIdealNmCandidate;
wheelPeakLimit = ...
    actuators.wheelTransmission.peakWheelTorqueIdealNmCandidate;

result.taskForce = taskForce;
result.outputAngles = outputAngles;
result.jointOutputTorqueNm = jointTorque;
result.jointRatedUtilization = abs(jointTorque) ./ jointRatedLimit;
result.jointPeakUtilization = abs(jointTorque) ./ jointPeakLimit;
result.withinJointRated = all(result.jointRatedUtilization <= 1);
result.withinJointPeak = all(result.jointPeakUtilization <= 1);
result.wheelTorqueNm = wheelTorqueNm;
result.wheelContinuousUtilizationIdeal = ...
    abs(wheelTorqueNm) / wheelContinuousLimit;
result.wheelPeakUtilizationIdeal = abs(wheelTorqueNm) / wheelPeakLimit;
result.withinWheelContinuousIdeal = ...
    result.wheelContinuousUtilizationIdeal <= 1;
result.withinWheelPeakIdeal = result.wheelPeakUtilizationIdeal <= 1;
result.withinAllRatedIdeal = ...
    result.withinJointRated && result.withinWheelContinuousIdeal;
result.withinAllPeakIdeal = ...
    result.withinJointPeak && result.withinWheelPeakIdeal;
result.vmc = vmc;
result.warning = [
    "关节侧按DM8009输出轴与机构输出1:1且不计损耗"
    "轮侧边界未乘外置减速箱效率，仅是理想上限"
    "本结果不能代替电流、温升、母线电压和动态转矩限制"
    ];
end
