function motor = dm_j8009_parameters()
%DM_J8009_PARAMETERS Manufacturer limits for the DM-J8009-2EC output shaft.

motor.model = "DM-J8009-2EC";
motor.teamAlias = "DM8009";
motor.sourcePdf = "重要参考资料/DM-J8009-2EC减速电机说明书V1.0.pdf";
motor.sourcePage = 8;
motor.ratedVoltageV = 24;
motor.supportedVoltageV = [24, 48];
motor.ratedCurrentA = 20;
motor.peakCurrentA = 50;
motor.recommendedProtectionCurrentA = 39;
motor.ratedOutputTorqueNm = 20;
motor.peakOutputTorqueNm = 40;
motor.ratedOutputSpeedRpm = [100, 200];
motor.noLoadOutputSpeedRpm = [160, 320];
motor.internalReductionRatio = 9;
motor.polePairs = 21;
motor.phaseInductanceH = 61e-6;
motor.phaseResistanceOhm = 0.09;
motor.massKg = 0.896;
motor.encoderBits = 14;
motor.encoderCount = 2;
motor.outputSingleTurnAbsolute = true;
motor.canBitrate = 1e6;
motor.uartBaudrate = 921600;
motor.controlModes = ["MIT", "velocity", "position"];
motor.mitTorqueRangeConfigurable = true;
motor.warning = "转矩和转速是减速器输出侧指标，不能再乘9；MIT的T_MAX需从设备配置读取";
end
