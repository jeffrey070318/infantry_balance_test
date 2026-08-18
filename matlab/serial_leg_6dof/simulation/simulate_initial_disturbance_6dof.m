function result = simulate_initial_disturbance_6dof(model, controller)
%SIMULATE_INITIAL_DISTURBANCE_6DOF Compare linear and nonlinear plants.

arguments
    model (1, 1) struct
    controller (1, 1) struct
end

time = (0:0.002:6).';
initialState = zeros(6, 1);
initialState(5) = deg2rad(2);
linearSystem = ss(model.A - model.B * controller.K, zeros(6, 1), ...
    eye(6), zeros(6, 1));
linearState = initial(linearSystem, initialState, time);

closedLoopDynamics = @(~, state) equivalent_leg_dynamics_6dof( ...
    state, -controller.K * state, model.parameters);
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
[nonlinearTime, nonlinearState] = ode45( ...
    closedLoopDynamics, time, initialState, options);
nonlinearInput = -(controller.K * nonlinearState.').';

result.time = time;
result.initialState = initialState;
result.linearState = linearState;
result.nonlinearTime = nonlinearTime;
result.nonlinearState = nonlinearState;
result.nonlinearInput = nonlinearInput;
result.maxStateDifference = max(abs(linearState - nonlinearState), [], 'all');
result.finalNonlinearStateNorm = norm(nonlinearState(end, :), inf);
result.finalAttitudeRateNorm = norm(nonlinearState(end, [1, 2, 4, 5, 6]), inf);
result.finalPositionError = nonlinearState(end, 3);
end
