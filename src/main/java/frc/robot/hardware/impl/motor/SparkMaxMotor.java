package frc.robot.hardware.impl.motor;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.hardware.Encoder;
import frc.robot.hardware.Hardware;
import frc.robot.hardware.Motor;
import frc.robot.hardware.impl.encoder.RevRelativeEncoder;

public record SparkMaxMotor(Hardware hardware, SparkMax sparkMax) implements Motor {
    @Override
    public void setVoltage(double volts) {
        sparkMax.setVoltage(volts);
    }

    @Override
    public double getMaxSpeed() {
        return 0; // not implemented yet
    }

    @Override
    public Encoder getEncoder() {
        return RevRelativeEncoder.of(hardware, sparkMax);
    }

    public static SparkMaxMotor of(Hardware hardware, int canId) {
        return new SparkMaxMotor(
                hardware,
                new SparkMax(canId, SparkLowLevel.MotorType.kBrushless)
        );
    }
}
