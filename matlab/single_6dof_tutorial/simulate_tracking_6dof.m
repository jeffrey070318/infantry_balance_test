function result = simulate_tracking_6dof(model, controller, reference)
%SIMULATE_TRACKING_6DOF Simulate full-state reference tracking with LQR K.
%
% u = K*(xd - x) leads to:
% xdot = (A - B*K)*x + B*K*xd.

arguments
    model (1, 1) struct
    controller (1, 1) struct
    reference (1, 1) struct
end

trackingSystem = ss(model.A - model.B * controller.K, ...
    model.B * controller.K, eye(6), zeros(6));
x = lsim(trackingSystem, reference.xd, reference.t, zeros(6, 1));
u = (controller.K * (reference.xd - x).').';

result.t = reference.t;
result.xd = reference.xd;
result.x = x;
result.u = u;
result.positionError = reference.position - x(:, 3);
result.velocityError = reference.velocity - x(:, 4);
result.maxLegAngle = max(abs(x(:, 1)));
result.maxBodyPitch = max(abs(x(:, 5)));
result.maxWheelTorque = max(abs(u(:, 1)));
result.maxPitchTorque = max(abs(u(:, 2)));
result.smallAngleLimit = deg2rad(15);
result.isWithinSmallAngleRange = ...
    result.maxLegAngle <= result.smallAngleLimit && ...
    result.maxBodyPitch <= result.smallAngleLimit;
end
