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
import frc.robot.Constants.Swerve;
import frc.robot.custom.TalonFXEncoder;

public class SwerveModule {

    private final PIDController turnPid;
    private final SimpleMotorFeedforward driveFeedforward;

    private final TalonFX driveMotor, turnMotor;
    private final Encoder driveEncoder;
    private final DutyCycleEncoder turnEncoder;

    public SwerveModule(Swerve.IDs ids) {
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
        driveMotor = new TalonFX(ids.driveMotorId);
        driveEncoder = new TalonFXEncoder(driveMotor);
        turnMotor = new TalonFX(ids.turnMotorId);
        turnEncoder = new DutyCycleEncoder(ids.turnEncoderId);
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
        return this.driveEncoder.getRate();
    }

    public void requestState(SwerveModuleState requested) {
        requested.optimize(getRot2d());
        requested.cosineScale(getRot2d());
        setDriveSpeed(requested.speedMetersPerSecond);
        setTurnPos(requested.angle.getRadians());
    }

    //m/s
    private void setDriveSpeed(double speed) {
        double volts = driveFeedforward.calculate(speed);
        this.driveMotor.setVoltage(volts);
    }

    private void setTurnPos(double rads) {
        double volts = turnPid.calculate(getTurnPosition(), rads);
        this.turnMotor.setVoltage(volts);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveEncoder.getDistance(), new Rotation2d(getTurnPosition())
        );
    }

    public SwerveModuleState getModuleState(){
        return null;
    }
}
