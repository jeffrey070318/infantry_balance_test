function result = task_force_to_joint_torque(taskForce, jointAngles, geometry)
%TASK_FORCE_TO_JOINT_TORQUE Apply tau = J' * [F0; Tp].

arguments
    taskForce (2, 1) double {mustBeFinite}
    jointAngles (2, 1) double {mustBeFinite}
    geometry (1, 1) struct
end

[jacobian, kinematics] = closed_chain_task_jacobian(jointAngles, geometry);
result.jointTorque = jacobian.' * taskForce;
result.taskForce = taskForce;
result.jacobian = jacobian;
result.kinematics = kinematics;
result.mapping = "tau_joint = J_Ltheta' * [F0; Tp]";
end
