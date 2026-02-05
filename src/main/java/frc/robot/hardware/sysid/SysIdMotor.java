package frc.robot.hardware.sysid;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.hardware.Encoder;
import frc.robot.hardware.Motor;
import frc.robot.hardware.impl.encoder.WPILIBDutyCycleEncoder;

public class SysIdMotor {
    private final RecordingMotor motor;
    private final Encoder encoder;
    private final String motorName;

    public SysIdMotor(String motorName, Motor motor) {
        this(motorName, motor, motor.getEncoder());
    }

    public SysIdMotor(String motorName, Motor motor, Encoder encoder) {
        this.motor = new RecordingMotor(motor);
        this.encoder = encoder;
        this.motorName = motorName;
    }

    public void setVoltage(Voltage voltage) {
        this.motor.setVoltage(voltage.in(Units.Volts));
    }

    public String motorName() {
        return this.motorName;
    }

    public void createDriveLog(SysIdRoutineLog.MotorLog log) {
        log.voltage(Voltage.ofBaseUnits(this.currentVoltage(), Units.Volts))
                .value("timestamp", Timer.getFPGATimestamp(), Units.Second.name())
                .linearPosition(Distance.ofBaseUnits(this.encoder.getPosition(), Units.Meters))
                .linearVelocity(LinearVelocity.ofBaseUnits(this.encoder.getVelocity(), Units.MetersPerSecond));
    }
    public void createTurnLog(SysIdRoutineLog.MotorLog log) {
        log.voltage(Voltage.ofBaseUnits(this.currentVoltage(), Units.Volts))
                .value("timestamp", Timer.getFPGATimestamp(), Units.Second.name())
                .angularPosition(Angle.ofBaseUnits(this.encoder.getPosition(), Units.Radians))
                .angularVelocity(AngularVelocity.ofBaseUnits(this.encoder.getVelocity(), Units.RadiansPerSecond));
    }

    public double currentVoltage() {
        return this.motor.getCurrentVoltage();
    }
}
