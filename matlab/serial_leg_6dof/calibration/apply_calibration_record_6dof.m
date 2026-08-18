function actuators = apply_calibration_record_6dof(record, actuators)
%APPLY_CALIBRATION_RECORD_6DOF Apply only a fully validated record.

arguments
    record (1, 1) struct
    actuators (1, 1) struct = serial_leg_actuator_parameters()
end

validation = validate_calibration_record_6dof(record);
if ~validation.ready
    error('sixdof:Calibration:RecordNotReady', ...
        '标定记录未通过，不能写入执行器参数：%s', ...
        strjoin(validation.blockers, '；'));
end

actuators.jointTransmission.rotationSign = record.joint.rotationSign;
actuators.jointTransmission.motorZeroRad = record.joint.motorZeroRad;
actuators.jointTransmission.outputZeroRad = record.joint.outputZeroRad;
actuators.jointTransmission.isCalibrated = true;
actuators.wheelTransmission.selectedExternalEfficiency = ...
    record.wheel.efficiency;
actuators.wheelTransmission.calibratedDirectionSign = ...
    record.wheel.directionSign;
actuators.wheelTransmission.calibratedTotalRatioMagnitude = ...
    record.wheel.motorToWheelRatioMagnitude;
actuators.wheelTransmission.isResolved = true;
actuators.imuCalibration = record.imu;
end
