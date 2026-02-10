package frc.robot.hardware.impl;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.hardware.Hardware;
import frc.robot.hardware.impl.encoder.KrakenEncoder;
import frc.robot.hardware.impl.encoder.RevRelativeEncoder;
import frc.robot.hardware.impl.encoder.WPILIBDutyCycleEncoder;
import frc.robot.hardware.impl.gyro.WPILIBGyro;
import frc.robot.hardware.impl.motor.KrakenMotor;
import frc.robot.hardware.impl.motor.SparkMaxMotor;

public class HardwareImpl {
    public static void register(Hardware hardware) {
        hardware.registerMotorType(TalonFX.class, KrakenMotor.class, KrakenMotor::of);
        hardware.registerEncoderType(TalonFX.class, KrakenEncoder.class, KrakenEncoder::of);
        hardware.registerEncoderType(DutyCycleEncoder.class, WPILIBDutyCycleEncoder.class, WPILIBDutyCycleEncoder::of);
        hardware.registerGyroType(ADXRS450_Gyro.class, WPILIBGyro.class, WPILIBGyro::of);
    }
}
