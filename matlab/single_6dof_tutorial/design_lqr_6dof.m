function controller = design_lqr_6dof(model)
%DESIGN_LQR_6DOF Reproduce the continuous-time LQR in the RM reference.
%
% The feedback law is u = -K*x for zero-reference regulation.

arguments
    model (1, 1) struct
end

controller.Q = diag([1, 1, 500, 100, 5000, 1]);
controller.R = diag([1, 0.25]);
[controller.K, controller.P, controller.closedLoopPoles] = ...
    lqr(model.A, model.B, controller.Q, controller.R);
end
