package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

public class SwerveModule extends SubsystemBase {

    private final PIDController turnPid;
    private final SimpleMotorFeedforward driveFeedforward;

    private final TalonFX driveMotor, turnMotor;
    private final DutyCycleEncoder turnEncoder;
    private final double offset;

    public SwerveModule(int driveMotorId, int turnMotorId, int turnEncoderId, double offset) {
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
        this.driveMotor = new TalonFX(driveMotorId);
        this.turnMotor = new TalonFX(turnMotorId);
        this.turnEncoder = new DutyCycleEncoder(turnEncoderId);

        this.offset = offset;
    }

    //Radians
    public Angle getTurnPosition() {
        double raw = Constants.Swerve.TO_DEGREES_FROM_RAW.apply(turnEncoder.get(), this.offset);
        //double raw = turnEncoder.get() * Constants.Swerve.ConversionFactor + this.offset;
        return Units.Degrees.of(raw);
    }
    public Rotation2d getRot2d() {
        return Rotation2d.fromRadians(this.getTurnPosition().in(Units.Radians));
    }
    // speed m/s
    public LinearVelocity getDriveSpeed() {
        return Units.MetersPerSecond.of(
            this.driveMotor.getVelocity().getValueAsDouble() / Constants.Drivetrain.MOTOR_ROTATIONS_PER_METER
        );
    }

    public void setDesiredState(SwerveModuleState desiredState){
        desiredState.optimize(getRot2d());
        setDriveSpeed(Units.MetersPerSecond.of(desiredState.speedMetersPerSecond));
        setTurnPosition(Units.Radians.of(desiredState.angle.getRadians()));
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModule");
        builder.addDoubleProperty("rawAngle", turnEncoder::get, null);
        builder.addDoubleProperty("angle", ()->getTurnPosition().in(Units.Degrees), (double mag)->setTurnPosition(Angle.ofBaseUnits(mag, null)));
        builder.addDoubleProperty("speed", ()->getDriveSpeed().magnitude(), (double mag)->setDriveSpeed(LinearVelocity.ofBaseUnits(mag, null)));
    }
}
