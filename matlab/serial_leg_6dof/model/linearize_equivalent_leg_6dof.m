function model = linearize_equivalent_leg_6dof(parameters, operatingState, operatingInput)
%LINEARIZE_EQUIVALENT_LEG_6DOF Central-difference linearization.

arguments
    parameters (1, 1) struct
    operatingState (6, 1) double {mustBeFinite} = zeros(6, 1)
    operatingInput (2, 1) double {mustBeFinite} = zeros(2, 1)
end

stateStep = 1e-6;
inputStep = 1e-6;
A = zeros(6);
B = zeros(6, 2);

for column = 1:6
    perturbation = zeros(6, 1);
    perturbation(column) = stateStep;
    plus = equivalent_leg_dynamics_6dof( ...
        operatingState + perturbation, operatingInput, parameters);
    minus = equivalent_leg_dynamics_6dof( ...
        operatingState - perturbation, operatingInput, parameters);
    A(:, column) = (plus - minus) / (2 * stateStep);
end

for column = 1:2
    perturbation = zeros(2, 1);
    perturbation(column) = inputStep;
    plus = equivalent_leg_dynamics_6dof( ...
        operatingState, operatingInput + perturbation, parameters);
    minus = equivalent_leg_dynamics_6dof( ...
        operatingState, operatingInput - perturbation, parameters);
    B(:, column) = (plus - minus) / (2 * inputStep);
end

model.A = A;
model.B = B;
model.C = eye(6);
model.D = zeros(6, 2);
model.parameters = parameters;
model.operatingState = operatingState;
model.operatingInput = operatingInput;
model.stateNames = ["theta", "theta_dot", "s", "s_dot", "phi", "phi_dot"];
model.stateUnits = ["rad", "rad/s", "m", "m/s", "rad", "rad/s"];
model.inputNames = ["T", "T_p"];
model.inputUnits = ["N*m", "N*m"];
model.linearizationMethod = "中心有限差分，不依赖 Symbolic Math Toolbox";
end
