# Single 6-DOF MATLAB Tutorial Design

## Goal

Build a minimal, reproducible MATLAB tutorial that teaches the complete linear-control workflow using the HIT Dream Wings RoboMaster six-state, two-input WLIP model at fixed leg length `L0 = 0.18 m`.

This is a RoboMaster reference-reproduction exercise. It is not the final model of the user's serial-leg robot.

## Model Contract

The state vector is fixed as:

```text
x = [theta; theta_dot; x_b; x_b_dot; phi; phi_dot]
```

- `theta`: equivalent leg angle, rad
- `theta_dot`: equivalent leg angular velocity, rad/s
- `x_b`: wheel-axis/body longitudinal position, m
- `x_b_dot`: longitudinal velocity, m/s
- `phi`: body pitch angle, rad
- `phi_dot`: body pitch angular velocity, rad/s

The input vector is:

```text
u = [T; T_p]
```

- `T`: wheel torque input used by the RM reference model, N*m
- `T_p`: equivalent leg-to-body pitch torque, N*m

The state definition and LQR framework are taken from pages 3-4 of `Robomaster平衡机器人系统控制-韭菜的菜.pdf`; page 7 provides the numerical `A`, `B`, `Q`, `R`, and `K` values for `L0 = 0.18 m`.

## Components

- `paper_model_6dof.m`: compatibility-named transcription of the RM reference `A`, `B`, state names, units, and source warning.
- `run_single_6dof.m`: inspect dimensions and poles, verify controllability, compute continuous-time LQR, simulate an initial body-pitch disturbance, and plot states and inputs.
- `tests/test_single_6dof.m`: verify dimensions, integrator row structure, exact published coefficients, controllability, unstable open loop, stable LQR loop, and numerical convergence.
- `README.md`: desktop MATLAB instructions and interpretation of every output.

## Simulation Scope

The initial exercise uses a two-degree body-pitch disturbance, zero reference, no saturation, no sensor noise, no discretization, no VMC, and no Simulink. This isolates the state-space and LQR workflow.

The next phase will reproduce the same model in Simulink. Robot-specific parameter derivation, left/right coupling, actuator limits, and the serial-leg VMC are deliberately excluded.

## Acceptance Criteria

- The model has dimensions `A: 6x6`, `B: 6x2`.
- `rank(ctrb(A,B)) == 6`.
- The open loop has at least one pole with positive real part.
- `lqr(A,B,Q,R)` produces a closed loop whose poles all have negative real parts.
- A two-degree body-pitch initial disturbance converges close to zero within the tutorial simulation window.
- All automated MATLAB tests pass.
