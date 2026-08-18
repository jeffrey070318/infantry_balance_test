function controller = design_heu_lqr_6dof(model)
%DESIGN_HEU_LQR_6DOF Reproduce the Q/R choice in HEU get_k_length.m.

arguments
    model (1, 1) struct
end

controller.Q = diag([20, 0.07, 1, 500, 3000, 0.3]);
controller.R = diag([20, 1]);
[controller.K, controller.P, controller.closedLoopPoles] = ...
    lqr(model.A, model.B, controller.Q, controller.R);
controller.feedbackLaw = "u = -K * (x - x_ref)";
end
