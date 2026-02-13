package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.SwerveConfig;

public class SwerveModule {

    private final PIDController turnPid;
    private final SimpleMotorFeedforward driveFeedforward;

    private final TalonFX driveMotor, turnMotor;
    private final DutyCycleEncoder turnEncoder;
    private final DutyCycleEncoder driveEncoder;
    private final State moduleState;

    public SwerveModule(SwerveConfig config) {
        // Control
        turnPid = new PIDController(
            Constants.Swerve.Control.TurnPID.kP, 
            Constants.Swerve.Control.TurnPID.kI,
            Constants.Swerve.Control.TurnPID.kD
        );
        driveFeedforward = new SimpleMotorFeedforward(
            Constants.Swerve.Control.DriveFeedforward.kS, 
            Constants.Swerve.Control.DriveFeedforward.kV
        );
        // Hardware
        driveMotor = new TalonFX(config.driveMotorId);
        driveEncoder = new DutyCycleEncoder(config.driveEncoderId);
        turnMotor = new TalonFX(config.turnMotorId);
        turnEncoder = new DutyCycleEncoder(config.turnEncoderId);
        moduleState = new State(null);
    }

    //Radians
    public double getTurnPosition() {
        return turnEncoder.get();
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
        requested.cosineScale(getRot2d());
        setDriveSpeed(requested.speedMetersPerSecond);
        setTurnPos(requested.angle.getRadians());
    }

    public void test() {
        //this.driveMotor.setVoltage(1);
        //this.turnMotor.setVoltage(1);
        this.setTurnPos(0);
    }

    //m/s
    private void setDriveSpeed(double speed) {
        double volts = this.context.calculate(speed, getDriveSpeed(), this.driveEncoder.getVelocity(), false);
        this.driveMotor.setVoltage(volts);
    }
    private void setTurnPos(double rads) {
        double volts = this.context.calculate(
            rads, getTurnPosition(), this.turnEncoder.getVelocity(), true
        );
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
