function reference = speed_reference_6dof(t, targetSpeed, stopTime)
%SPEED_REFERENCE_6DOF Build the RM reference velocity and position command.

arguments
    t (:, 1) double {mustBeNonnegative, mustBeFinite}
    targetSpeed (1, 1) double {mustBeFinite} = 1.5
    stopTime (1, 1) double {mustBePositive, mustBeFinite} = 3.0
end

assert(issorted(t), 'Time samples must be sorted in ascending order.');

velocity = targetSpeed * double(t > 0 & t < stopTime);
position = targetSpeed * min(t, stopTime);

xd = zeros(numel(t), 6);
xd(:, 3) = position;
xd(:, 4) = velocity;

reference.t = t;
reference.xd = xd;
reference.position = position;
reference.velocity = velocity;
reference.targetSpeed = targetSpeed;
reference.stopTime = stopTime;
end
