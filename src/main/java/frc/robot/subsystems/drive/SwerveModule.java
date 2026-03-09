package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    public Angle getTurnPosition() {
        return Units.Degrees.ofBaseUnits(turnEncoder.get());
    }
    public Rotation2d getRot2d() {
        return Rotation2d.fromRadians(this.getTurnPosition().in(Units.Radians));
    }
    // speed m/s
    public LinearVelocity getDriveSpeed() {
        return Units.MetersPerSecond.ofBaseUnits(
            this.driveMotor.getVelocity().getValueAsDouble() / Constants.Drivetrain.MOTOR_ROTATIONS_PER_METER
        );
    }

    public void setDesiredState(SwerveModuleState desiredState){
        desiredState.optimize(getRot2d());
        setDriveSpeed(Units.MetersPerSecond.ofBaseUnits(desiredState.speedMetersPerSecond));
        setTurnPosition(Units.Radians.ofBaseUnits(desiredState.angle.getRadians()));
    }

    //m/s
    private void setDriveSpeed(LinearVelocity speed) {
        double mps = speed.in(Units.MetersPerSecond);
        double volts = driveFeedforward.calculate(mps);
        this.driveMotor.setVoltage(volts);
    }
    
    private void setTurnPosition(Angle angle) {
        double rads = angle.in(Units.Radians);
        double volts = turnPid.calculate(getTurnPosition().in(Units.Radians), rads);
        this.turnMotor.setVoltage(volts);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveMotor.getPosition().getValueAsDouble(), new Rotation2d(getTurnPosition())
        );
    }
}
