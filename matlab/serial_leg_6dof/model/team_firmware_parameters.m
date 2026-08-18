function parameters = team_firmware_parameters()
%TEAM_FIRMWARE_PARAMETERS Confirmed geometry and runtime limits from firmware.

parameters.gravity = 9.791;
parameters.wheelRadius = 0.060;
parameters.wheelDistance = 0.435;
parameters.legLengthDefault = 0.300;
parameters.legLengthRange = [0.150; 0.320];
parameters.maxLegLengthRate = 0.200;
parameters.bodyMassForSupportFeedforward = 5.0;
parameters.wheelMassForGroundDetection = 0.3;
parameters.firmwareReductionRatio = 15.7;
parameters.firmwareEfficiencyConstant = 0.1;
parameters.source = [
    "balance_infantry/Robot/RoboTask/chassis/chassis_def.h"
    "balance_infantry/Robot/Controller/kinematics.c"
    "balance_infantry/Robot/RoboTask/chassis/chassis.c"
    ];
parameters.geometryContract = [
    "L0是轮心到髋轴的等效腿长，不是LEG1到LEG4中的某一根连杆"
    "Phi1/Phi4是两个主动输出角，固件用pi/2加电机角得到"
    ];
parameters.warning = [
    "bodyMassForSupportFeedforward只对应固件腿长支撑前馈，不等于完整六维动力学M"
    "减速效率常量和质量定义仍需实测或总成质量属性确认"
    ];
end
