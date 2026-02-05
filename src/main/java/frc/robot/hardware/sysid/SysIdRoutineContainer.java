package frc.robot.hardware.sysid;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.Encoder;
import frc.robot.hardware.Motor;
import frc.robot.util.DummySubsystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

public class SysIdRoutineContainer {
    private final Map<String, RoutineMotor> motors;

    private final List<SysIdRoutineLog> logs;

    private final SysIdRoutine routine;

    public SysIdRoutineContainer() {
        this.motors = new HashMap<>();
        this.logs = new ArrayList<>();

        this.routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(this::driveMotors, this::listen, new DummySubsystem())
        );
    }

    public SysIdRoutineContainer addDriveMotor(String motorName, Motor motor) {
        this.motors.put(
                motorName,
                new RoutineMotor(
                        new SysIdMotor(motorName, motor),
                        false
                )
        );

        return this;
    }

    public SysIdRoutineContainer addTurnMotor(String motorName, Motor motor, Encoder turnEncoder) {
        this.motors.put(
                motorName,
                new RoutineMotor(
                        new SysIdMotor(motorName, motor, turnEncoder),
                        true
                )
        );

        return this;
    }

    public Command runRoutine(RoutineType type, Direction dir) {
        return switch(type) {
            case QUASISTATIC -> this.routine.quasistatic(dir.asWpilibDirection());
            case DYNAMIC -> this.routine.dynamic(dir.asWpilibDirection());
        };
    }

    // Makes sure the routine is able to run safely, only if there are motors
    public boolean valid() {
        return !this.motors.isEmpty();
    }

    public SubRoutineResults consolidateResults() {
        return null;
    }

    // Routine Wrappers

    private void driveMotors(Voltage voltage) {
        motors.values().forEach(motor -> motor.sysIdMotor().setVoltage(voltage));
    }

    private void listen(SysIdRoutineLog log) {
        populateLog(log);
        this.logs.add(log);
    }

    private void populateLog(SysIdRoutineLog log) {
        motors.forEach((motorName, motor) -> {
            SysIdRoutineLog.MotorLog motorLog = log.motor(motorName);
            if(motor.isTurn()) motor.sysIdMotor().createTurnLog(motorLog);
            else motor.sysIdMotor().createDriveLog(motorLog);
        });
    }

    public record BuiltSubRoutine(Supplier<Command> run) {
        public static BuiltSubRoutine build(SysIdRoutineContainer routine, RoutineType type, Direction dir) {
            return new BuiltSubRoutine(
                    () -> routine.runRoutine(type, dir)
            );
        }
    }

    public record SubRoutineResults() {

    }

    record RoutineMotor(SysIdMotor sysIdMotor, boolean isTurn) {}

    public enum RoutineType {
        QUASISTATIC,
        DYNAMIC
    }

    public enum Direction {
        FORWARD(SysIdRoutine.Direction.kForward),
        BACKWARD(SysIdRoutine.Direction.kReverse);

        private final SysIdRoutine.Direction direction;

        Direction(SysIdRoutine.Direction dir) {
            this.direction = dir;
        }

        public SysIdRoutine.Direction asWpilibDirection() {
            return this.direction;
        }
    }
}
