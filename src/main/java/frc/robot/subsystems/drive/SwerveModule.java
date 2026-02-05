package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.hardware.Encoder;
import frc.robot.hardware.Hardware;
import frc.robot.hardware.Motor;
import frc.robot.hardware.sysid.SysIdRoutineContainer;
import frc.robot.subsystems.drive.control.ControlContext;
import frc.robot.subsystems.drive.control.LQRControlContext;
import frc.robot.subsystems.drive.control.PidControlContext;

public class SwerveModule {
    private final ControlContext context;

    private final Motor driveMotor, turnMotor;
    private final Encoder driveEncoder, turnEncoder;

    private final State moduleState;

    public SwerveModule(Hardware hardware, Constants.SwerveModuleConstants constants) {
        this.context = switch(Constants.Swerve.CONTROLLER_TYPE) {
            case LQR -> new LQRControlContext();
            case PID -> new PidControlContext();
        };

        Constants.SwerveModuleHardware hw = constants.createHardware(hardware);
        this.driveMotor = hw.driveMotor();
        this.driveEncoder = this.driveMotor.getEncoder();
        this.turnMotor = hw.turnMotor();
        this.turnEncoder = hw.turnEncoder();

        this.moduleState = new State(this);
    }

    //Radians
    public double getTurnPosition() {
        return this.turnEncoder.getPosition();
    }
    public Rotation2d getRot2d() {
        return Rotation2d.fromRadians(this.getTurnPosition());
    }
    // speed m/s
    public double getDriveSpeed() {
        return this.driveEncoder.getVelocity();
    }
    
    public State getModuleState() {
        return this.moduleState;
    }

    public double getMaxSpeed() {
        return this.driveMotor.getMaxSpeed();
    }

    public void requestState(SwerveModuleState requested) {
        requested.optimize(getRot2d());
        requested.speedMetersPerSecond *= requested.angle.minus(getRot2d()).getCos();
        setDriveSpeed(requested.speedMetersPerSecond);
        setTurnPos(requested.angle.getRadians());
    }

    public void populateContainer(String prefix, SysIdRoutineContainer container) {
        container.addDriveMotor(prefix + "D", this.driveMotor)
                .addTurnMotor(prefix + "T", this.turnMotor, this.turnEncoder);
    }

    //m/s
    private void setDriveSpeed(double speed) {
        double volts = this.context.calculate(speed, getDriveSpeed(), false);
        this.driveMotor.setVoltage(volts);
    }
    private void setTurnPos(double rads) {
        double volts = this.context.calculate(rads, getTurnPosition(), true);
        this.turnMotor.setVoltage(volts);
    }

    private Encoder getDriveEncoder() {
        return driveEncoder;
    }

    public record State(SwerveModule module) {
        public SwerveModulePosition asPosition() {
            return new SwerveModulePosition(
                    module.getDriveEncoder().getPosition(),
                    module.getRot2d()
            );
        }
        public SwerveModuleState asState() {
            return new SwerveModuleState(
                    module.getDriveSpeed(),
                    module.getRot2d()
            );
        }
    }
}
