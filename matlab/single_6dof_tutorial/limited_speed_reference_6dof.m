function reference = limited_speed_reference_6dof(t, targetSpeed, cruiseEndTime, maxAcceleration)
%LIMITED_SPEED_REFERENCE_6DOF Build an acceleration-limited speed command.
%
% The command accelerates from zero, cruises, and then decelerates to zero.
% Position is the analytic integral of that trapezoidal velocity command.

arguments
    t (:, 1) double {mustBeNonnegative, mustBeFinite}
    targetSpeed (1, 1) double {mustBePositive, mustBeFinite} = 1.5
    cruiseEndTime (1, 1) double {mustBePositive, mustBeFinite} = 3.0
    maxAcceleration (1, 1) double {mustBePositive, mustBeFinite} = 1.0
end

assert(issorted(t), 'Time samples must be sorted in ascending order.');

rampTime = targetSpeed / maxAcceleration;
assert(cruiseEndTime >= rampTime, ...
    'cruiseEndTime must not be earlier than the end of acceleration.');
stopTime = cruiseEndTime + rampTime;

velocity = zeros(size(t));
position = zeros(size(t));

accelerating = t < rampTime;
cruising = t >= rampTime & t < cruiseEndTime;
decelerating = t >= cruiseEndTime & t < stopTime;
stopped = t >= stopTime;

velocity(accelerating) = maxAcceleration * t(accelerating);
position(accelerating) = 0.5 * maxAcceleration * t(accelerating).^2;

accelerationDistance = 0.5 * targetSpeed * rampTime;
velocity(cruising) = targetSpeed;
position(cruising) = accelerationDistance + ...
    targetSpeed * (t(cruising) - rampTime);

decelerationTime = t(decelerating) - cruiseEndTime;
velocity(decelerating) = targetSpeed - maxAcceleration * decelerationTime;
positionAtDeceleration = accelerationDistance + ...
    targetSpeed * (cruiseEndTime - rampTime);
position(decelerating) = positionAtDeceleration + ...
    targetSpeed * decelerationTime - ...
    0.5 * maxAcceleration * decelerationTime.^2;

finalPosition = targetSpeed * cruiseEndTime;
position(stopped) = finalPosition;

xd = zeros(numel(t), 6);
xd(:, 3) = position;
xd(:, 4) = velocity;

reference.t = t;
reference.xd = xd;
reference.position = position;
reference.velocity = velocity;
reference.targetSpeed = targetSpeed;
reference.maxAcceleration = maxAcceleration;
reference.rampTime = rampTime;
reference.cruiseEndTime = cruiseEndTime;
reference.stopTime = stopTime;
reference.finalPosition = finalPosition;
end
