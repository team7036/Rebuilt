package frc.robot.hardware.sysid;

import frc.robot.hardware.Encoder;
import frc.robot.hardware.Motor;

public class RecordingMotor implements Motor {
    private final Motor motor;

    private double currentVoltage;

    RecordingMotor(Motor motor) {
        this.motor = motor;
    }
    @Override
    public void setVoltage(double volts) {
        this.currentVoltage = volts;
        this.motor.setVoltage(volts);
    }

    @Override
    public double getMaxSpeed() {
        return this.motor.getMaxSpeed();
    }

    @Override
    public Encoder getEncoder() {
        return this.motor.getEncoder();
    }

    public double getCurrentVoltage() {
        return currentVoltage;
    }
}
