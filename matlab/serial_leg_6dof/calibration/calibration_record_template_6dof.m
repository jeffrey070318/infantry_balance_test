function record = calibration_record_template_6dof()
%CALIBRATION_RECORD_TEMPLATE_6DOF Return an intentionally unfilled record.

record.version = "sixdof-calibration-v1";
record.timestamp = "";
record.operator = "";
record.firmwareCommit = "";
record.batteryVoltageV = NaN;
record.referenceLegLengthM = 0.306802737;
record.referenceOutputAnglesRad = deg2rad([38.174; 141.823]);
record.joint.rotationSign = [NaN; NaN];
record.joint.motorZeroRad = [NaN; NaN];
record.joint.outputZeroRad = [NaN; NaN];
record.joint.outputErrorRad = [NaN; NaN];
record.joint.isCalibrated = false;
record.imu.rightPitchSign = NaN;
record.imu.leftPitchSign = NaN;
record.imu.rightGyroYSign = NaN;
record.imu.leftGyroYSign = NaN;
record.imu.staticGyroRmsRadPerS = NaN;
record.imu.isCalibrated = false;
record.wheel.motorToWheelRatioMagnitude = NaN;
record.wheel.directionSign = NaN;
record.wheel.efficiency = NaN;
record.wheel.isResolved = false;
record.rawDataFields = [
    "timestamp_s"
    "side"
    "motor_id"
    "motor_angle_rad"
    "output_angle_rad"
    "motor_velocity_rad_s"
    "output_velocity_rad_s"
    "torque_command_Nm"
    "measured_current_A"
    "imu_pitch_rad"
    "imu_gyro_y_rad_s"
    "wheel_speed_rad_s"
    "wheel_direction_sign"
    ];
record.warning = "空模板；任何NaN或false都表示尚未完成标定。";
end
