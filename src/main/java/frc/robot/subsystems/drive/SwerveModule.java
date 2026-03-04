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

public class SwerveModule {

    private final PIDController turnPid;
    private final SimpleMotorFeedforward driveFeedforward;

    private final TalonFX driveMotor, turnMotor;
    private final DutyCycleEncoder turnEncoder;

    public SwerveModule(int driveMotorId, int turnMotorId, int turnEncoderId) {
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
        driveMotor = new TalonFX(driveMotorId);
        turnMotor = new TalonFX(turnMotorId);
        turnEncoder = new DutyCycleEncoder(turnEncoderId);
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
        return this.driveMotor.getVelocity().getValueAsDouble();
    }

    public void setDesiredState(SwerveModuleState desiredState){
        desiredState.optimize(getRot2d());
        setDriveSpeed(desiredState.speedMetersPerSecond);
        setTurnPosition(desiredState.angle.getRadians());
    }

    //m/s
    private void setDriveSpeed(double speed) {
        double volts = driveFeedforward.calculate(speed);
        this.driveMotor.setVoltage(volts);
    }

    private void setTurnPosition(double rads) {
        double volts = turnPid.calculate(getTurnPosition(), rads);
        this.turnMotor.setVoltage(volts);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveMotor.getPosition().getValueAsDouble(), new Rotation2d(getTurnPosition())
        );
    }
}
