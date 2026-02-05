package frc.robot.hardware.impl.motor;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.hardware.Encoder;
import frc.robot.hardware.Hardware;
import frc.robot.hardware.Motor;
import frc.robot.hardware.impl.encoder.KrakenEncoder;

public record KrakenMotor(TalonFX fx) implements Motor {

    private static final VoltageOut VOLTAGE_REQUEST = new VoltageOut(0.0);

    @Override
    public void setVoltage(double volts) {
        fx.setControl(VOLTAGE_REQUEST.withOutput(volts));
    }

    @Override
    public double getMaxSpeed() {
        return KrakenEncoder.MAX_WHEEL_SPEED_MPS;
    }

    @Override
    public Encoder getEncoder() {
        return new KrakenEncoder(fx);
    }

    public static KrakenMotor of(Hardware hardware, int canId) {
        return new KrakenMotor(
                new TalonFX(canId)
        );
    }
}
