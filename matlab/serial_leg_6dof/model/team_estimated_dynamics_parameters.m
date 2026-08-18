function parameters = team_estimated_dynamics_parameters( ...
    totalMass, legLength, legMassEach, bodyComOffset)
%TEAM_ESTIMATED_DYNAMICS_PARAMETERS Provisional 18-20 kg team model.

arguments
    totalMass (1, 1) double {mustBeFinite, mustBePositive} = 19
    legLength (1, 1) double {mustBeFinite, mustBePositive} = 0.30
    legMassEach (1, 1) double {mustBeFinite, mustBePositive} = 1.8
    bodyComOffset (1, 1) double {mustBeFinite, mustBePositive} = 0.06
end

firmware = team_firmware_parameters();
wheelMassEach = firmware.wheelMassForGroundDetection;
bodyMass = totalMass - 2 * (wheelMassEach + legMassEach);
if bodyMass <= 0
    error('sixdof:Parameters:InvalidMassSplit', ...
        '总质量不足以覆盖左右轮和腿的暂估质量。');
end

parameters.legLength = legLength;
parameters.wheelRadius = firmware.wheelRadius;
parameters.lowerComDistance = legLength / 2;
parameters.upperComDistance = legLength / 2;
parameters.bodyComOffset = bodyComOffset;
parameters.wheelMass = wheelMassEach;
parameters.legMass = legMassEach;
parameters.bodyMass = bodyMass;

% First-pass geometric estimates; sensitivity matters more than exact values.
parameters.wheelInertia = wheelMassEach * parameters.wheelRadius^2;
parameters.legInertia = legMassEach * (legLength^2 + 0.10^2) / 12;
parameters.bodyInertia = bodyMass * (0.45^2 + 0.25^2) / 12;
parameters.gravity = firmware.gravity;

parameters.totalMassDefinition = ...
    "totalMass = bodyMass + 2*(legMass + wheelMass)";
parameters.totalMass = totalMass;
parameters.massClosureResidual = bodyMass + ...
    2 * (legMassEach + wheelMassEach) - totalMass;
parameters.isProvisional = true;
parameters.source = [
    "用户指定整车总质量按18~20 kg估算"
    "balance_infantry固件提供轮质量0.3 kg/侧和轮半径0.06 m"
    "腿质量、质心偏置和惯量为第一版工程暂估"
    ];
parameters.warning = [
    "只用于固定腿长六维仿真和质量灵敏度验证"
    "不得把本参数生成的K直接下发实机"
    ];
end
