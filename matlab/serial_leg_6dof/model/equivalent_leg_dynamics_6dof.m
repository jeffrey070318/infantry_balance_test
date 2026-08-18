function [stateDerivative, diagnostics] = equivalent_leg_dynamics_6dof(state, input, parameters)
%EQUIVALENT_LEG_DYNAMICS_6DOF Evaluate the HEU equivalent-leg equations.

arguments
    state (6, 1) double {mustBeFinite}
    input (2, 1) double {mustBeFinite}
    parameters (1, 1) struct
end

zeroAcceleration = zeros(3, 1);
bias = accelerationResidual(zeroAcceleration, state, input, parameters);
massMatrix = zeros(3);
for column = 1:3
    unitAcceleration = zeros(3, 1);
    unitAcceleration(column) = 1;
    massMatrix(:, column) = ...
        accelerationResidual(unitAcceleration, state, input, parameters) - bias;
end

if rcond(massMatrix) < 1e-10
    error('sixdof:Dynamics:SingularMassMatrix', ...
        '等效腿动力学质量矩阵接近奇异，当前状态不能可靠求解。');
end

acceleration = -(massMatrix \ bias);
stateDerivative = [state(2); acceleration(1); state(4); ...
    acceleration(2); state(6); acceleration(3)];

if nargout > 1
    diagnostics.massMatrix = massMatrix;
    diagnostics.bias = bias;
    diagnostics.acceleration = acceleration;
    diagnostics.residual = accelerationResidual( ...
        acceleration, state, input, parameters);
    diagnostics.reciprocalCondition = rcond(massMatrix);
end
end

function residual = accelerationResidual(qdd, state, input, p)
theta = state(1);
thetaDot = state(2);
phi = state(5);
phiDot = state(6);
thetaDDot = qdd(1);
xDDot = qdd(2);
phiDDot = qdd(3);
T = input(1);
Tp = input(2);

R = p.wheelRadius;
L = p.lowerComDistance;
LM = p.upperComDistance;
l = p.bodyComOffset;
mw = p.wheelMass;
mp = p.legMass;
M = p.bodyMass;

bodyXDDot = xDDot + (L + LM) * ...
    (cos(theta) * thetaDDot - sin(theta) * thetaDot^2) - ...
    l * (cos(phi) * phiDDot - sin(phi) * phiDot^2);
bodyZDDot = -(L + LM) * ...
    (sin(theta) * thetaDDot + cos(theta) * thetaDot^2) - ...
    l * (sin(phi) * phiDDot + cos(phi) * phiDot^2);
legXDDot = xDDot + L * ...
    (cos(theta) * thetaDDot - sin(theta) * thetaDot^2);
legZDDot = -L * ...
    (sin(theta) * thetaDDot + cos(theta) * thetaDot^2);

NM = M * bodyXDDot;
N = NM + mp * legXDDot;
PM = M * p.gravity + M * bodyZDDot;
P = PM + mp * p.gravity + mp * legZDDot;

residual = [
    xDDot - (T - N * R) / (p.wheelInertia / R + mw * R);
    p.legInertia * thetaDDot - ((P * L + PM * LM) * sin(theta) - ...
        (N * L + NM * LM) * cos(theta) - T + Tp);
    p.bodyInertia * phiDDot - ...
        (Tp + NM * l * cos(phi) + PM * l * sin(phi))];
end
