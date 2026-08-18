function controller = evaluate_team_gain_schedule_6dof(schedule, legLength)
%EVALUATE_TEAM_GAIN_SCHEDULE_6DOF Evaluate the fitted provisional K(L0).

arguments
    schedule (1, 1) struct
    legLength (1, 1) double {mustBeFinite, mustBePositive}
end

if legLength < schedule.validRange(1) || legLength > schedule.validRange(2)
    error('sixdof:GainSchedule:LengthOutOfRange', ...
        'L0=%.6f m超出K(L0)有效范围[%.6f, %.6f] m。', ...
        legLength, schedule.validRange(1), schedule.validRange(2));
end

gainVector = schedule.coefficients * ...
    [legLength^3; legLength^2; legLength; 1];
controller.K = reshape(gainVector, 6, 2).';
controller.legLength = legLength;
controller.feedbackLaw = "u = -K(L0) * (x - x_ref)";
controller.source = "build_team_gain_schedule_6dof三次拟合";
end
