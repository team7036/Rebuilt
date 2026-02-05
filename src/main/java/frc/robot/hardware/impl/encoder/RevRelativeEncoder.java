package frc.robot.hardware.impl.encoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.hardware.Encoder;
import frc.robot.hardware.Hardware;

public record RevRelativeEncoder(RelativeEncoder encoder) implements Encoder {

    private static final double WHEEL_DIAMETER_METERS = 0.1016;
    private static final double WHEEL_CIRCUMFERENCE_METERS =
            Math.PI * WHEEL_DIAMETER_METERS;

    private static final double DRIVE_GEAR_RATIO = 6.12;

    private static final double MOTOR_ROTATIONS_PER_METER =
            DRIVE_GEAR_RATIO / WHEEL_CIRCUMFERENCE_METERS;

    @Override
    public double getPosition() {
        return encoder.getPosition() / MOTOR_ROTATIONS_PER_METER;
    }

    @Override
    public double getVelocity() {
        double motorRPS = encoder.getVelocity() / 60.0;
        return motorRPS / MOTOR_ROTATIONS_PER_METER;
    }

    public static RevRelativeEncoder of(Hardware hardware, int canId) {
        return of(hardware, new SparkMax(canId, SparkLowLevel.MotorType.kBrushless));
    }

    public static RevRelativeEncoder of(Hardware hardware, SparkMax sparkMax) {
        return new RevRelativeEncoder(sparkMax.getEncoder());
    }
}
