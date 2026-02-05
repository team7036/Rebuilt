package frc.robot.hardware.impl.encoder;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.hardware.Encoder;
import frc.robot.hardware.Hardware;

public record KrakenEncoder(TalonFX encoder) implements Encoder {
    private static final double FREE_SPEED_RPM = 6000.0; //rpm
    private static final double FREE_SPEED_RPS = FREE_SPEED_RPM / 60.0; // 100 rps

    private static final double WHEEL_DIAMETER_METERS = 0.1016; //meters, 4 inches
    private static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS; //meters
    private static final double DRIVE_GEAR_RATIO = 6.12;

    public static final double MAX_WHEEL_SPEED_MPS =
            FREE_SPEED_RPS / DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE_METERS;

    private static final double MOTOR_ROTATIONS_PER_METER =
            DRIVE_GEAR_RATIO / WHEEL_CIRCUMFERENCE_METERS;

    @Override
    public double getPosition() {
        return this.encoder.getPosition().getValueAsDouble() / MOTOR_ROTATIONS_PER_METER;
    }

    @Override
    public double getVelocity() {
        return this.encoder.getVelocity().getValueAsDouble() / MOTOR_ROTATIONS_PER_METER;
    }

    public static KrakenEncoder of(Hardware hardware, int canId) {
        return new KrakenEncoder(
                new TalonFX(canId)
        );
    }
}
