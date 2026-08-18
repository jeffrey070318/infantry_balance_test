function model = paper_model_6dof()
%PAPER_MODEL_6DOF HIT Dream Wings RM six-state model for learning.
%
% This model follows the RoboMaster technical article archived as:
%   Robomaster平衡机器人系统控制-韭菜的菜.pdf
% Pages 3-4 define the six-state WLIP/LQR framework, and page 7 gives the
% A/B/Q/R/K values at L0 = 0.18 m. It is not a model of the user's serial-leg
% robot. Do not transfer its gains or coefficients to hardware.
%
% State vector:
%   x = [theta; theta_dot; x_b; x_b_dot; phi; phi_dot]
%
% Input vector:
%   u = [T; T_p]

model.A = [
      0,       1, 0, 0,        0, 0;
    265.9556,  0, 0, 0,  80.6327, 0;
      0,       0, 0, 1,        0, 0;
    -25.4562,  0, 0, 0,   1.8637, 0;
      0,       0, 0, 0,        0, 1;
    156.6952,  0, 0, 0, 183.0614, 0];

model.B = [
      0,       0;
    -15.1389, 13.8563;
      0,       0;
      2.1208, -0.7158;
      0,       0;
     -4.2238, 16.8001];

model.C = eye(6);
model.D = zeros(6, 2);

model.stateNames = ["theta", "theta_dot", "x_b", "x_b_dot", ...
    "phi", "phi_dot"];
model.stateUnits = ["rad", "rad/s", "m", "m/s", "rad", "rad/s"];
model.inputNames = ["T", "T_p"];
model.inputUnits = ["N*m", "N*m"];
model.legLength = 0.18;
model.source = "HIT Dream Wings RoboMaster six-state WLIP, fixed L0 = 0.18 m";
model.warning = "RoboMaster reference reproduction only; not a robot-specific model.";
end
