function controller = design_opensource_lqr_6dof(model)
%DESIGN_OPENSOURCE_LQR_6DOF Design LQR using this open-source model's Q/R.

arguments
    model (1, 1) struct
end

controller.Q = diag([1, 1, 20, 5, 50, 1]);
controller.R = diag([1, 0.25]);
[controller.K, controller.P, controller.closedLoopPoles] = ...
    lqr(model.A, model.B, controller.Q, controller.R);
end
