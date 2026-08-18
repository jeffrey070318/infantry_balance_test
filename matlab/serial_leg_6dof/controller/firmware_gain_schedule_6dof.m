function schedule = firmware_gain_schedule_6dof(legLength, takeoff)
%FIRMWARE_GAIN_SCHEDULE_6DOF Evaluate active balance_infantry K polynomials.

arguments
    legLength (1, 1) double {mustBeFinite, mustBePositive}
    takeoff (1, 1) logical = false
end

% Rows are [a3 a2 a1 a0] for K11..K16,K21..K26.
coefficients = [
    -93.527275658083298, 105.5382650787542, -68.356866413897464, -0.477530048051153
      1.870315711240097,  -2.786991369786322, -7.382019971866752,  0.147467682714690
    -52.775375871623346, 48.453886159940289, -15.802651205697718, -2.520415249308310
    -48.109067270853139, 45.760599562362160, -17.612639543180229, -2.313535434564870
   -204.059082854889900, 210.48819351095282, -83.624725594734372, 15.579492486342044
     -9.974748215683954, 10.759322752665343, -4.598613345996796,  0.997248049706160
     62.490876763095592, -46.074184930502916, 5.777824802944528,  6.392059376030146
     12.932372608353148, -12.420045180605342, 4.523406550942889,  0.444600881571619
   -120.242967708718100, 124.03135749360770, -49.276342119240752, 9.180303987126345
   -103.042453767210000, 106.82567768395490, -42.991678906401674, 8.518622836271815
    373.178261587250700, -342.62071470140140, 111.74161828308140, 17.822027141896108
     22.936850970198119, -21.426162296531761, 7.187111998995120,  0.272352562192785
    ];

gainVector = coefficients * [legLength^3; legLength^2; legLength; 1];
K = reshape(gainVector, 6, 2).';
if takeoff
    K(1, :) = 0;
    K(2, 3:6) = 0;
end

firmware = team_firmware_parameters();
schedule.K = K;
schedule.legLength = legLength;
schedule.takeoff = takeoff;
schedule.coefficients = coefficients;
schedule.validRange = firmware.legLengthRange;
schedule.stateOrder = ["theta", "theta_dot", "x", "x_dot", "phi", "phi_dot"];
schedule.outputOrder = ["wheel_torque_T", "leg_angle_torque_Tp"];
schedule.source = ...
    "balance_infantry/Robot/Controller/kinematics.c active a11..a26";
schedule.firmwareExpression = "固件源码先计算u=K*x，再经过左右侧电机方向映射";
schedule.matlabAuditedExpression = "当前MATLAB坐标中按u=-K*x验证";
schedule.warning = [
    "K数值可交叉验证，实机最终符号仍由左右轮和四个关节电机标定决定"
    "静态LQR_K_L/R数组会被Calc_LQR_K每周期覆盖，不作为当前活动K"
    ];
end
