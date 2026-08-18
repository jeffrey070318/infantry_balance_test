function motor = m3508_c620_parameters()
%M3508_C620_PARAMETERS Manual values for M3508 used with the C620 ESC.

motor.model = "RoboMaster M3508 P19";
motor.controller = "RoboMaster C620";
motor.teamAlias = "DJ3508";
motor.sourcePdf = "重要参考资料/RoboMaster M3508直流无刷减速电机使用说明V1.0（中英日）.pdf";
motor.sourcePages = [12, 13];
motor.ratedVoltageV = 24;
motor.noLoadOutputSpeedRpm = 482;
motor.ratedOutputSpeedRpm = 469;
motor.ratedOutputTorqueNm = 3;
motor.stallOutputTorqueNm = 4.5;
motor.ratedInputCurrentA = 10;
motor.manualStallInputCurrentA = 2.5;
motor.maxEfficiency = 0.70;
motor.outputTorqueConstantNmPerA = 0.3;
motor.outputSpeedConstantRpmPerV = 24.48;
motor.speedTorqueGradientRpmPerNm = 72;
motor.mechanicalTimeConstantS = 0.049;
motor.phaseResistanceOhm = 0.194;
motor.phaseInductanceH = 0.097e-3;
motor.polePairs = 7;
motor.maxRadialDynamicLoadN = 210;
motor.massKg = 0.365;
motor.internalReductionRatio = 3591 / 187;
motor.warning = [
    "手册第12页的堵转电流2.5A小于额定电流10A，数据自相矛盾"
    "堵转电流不得用于保护阈值；3N*m连续、4.5N*m堵转转矩仅作手册边界"
    ];
end
