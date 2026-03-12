package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

    private final PIDController turnPid, drivePid;
    private final SimpleMotorFeedforward driveFeedforward, turnFeedforward;

    private final TalonFX driveMotor, turnMotor;
    private final DutyCycleEncoder turnEncoder;

    public SwerveModule(String moduleName, int driveMotorId, int turnMotorId, int turnEncoderId, double offset) {
        
        // Control
        this.turnPid = new PIDController(
                Constants.Swerve.PID.Turn.kP,
                Constants.Swerve.PID.Turn.kI,
                Constants.Swerve.PID.Turn.kD);
        this.turnPid.enableContinuousInput(-Math.PI, Math.PI);
        this.turnFeedforward = new SimpleMotorFeedforward(
            Constants.Swerve.Feedforward.Turn.kS,
            Constants.Swerve.Feedforward.Turn.kV
        );
        this.drivePid = new PIDController(
                Constants.Swerve.PID.Drive.kP,
                Constants.Swerve.PID.Drive.kI,
                Constants.Swerve.PID.Drive.kD);
        this.driveFeedforward = new SimpleMotorFeedforward(
                Constants.Swerve.Feedforward.Drive.kS,
                Constants.Swerve.Feedforward.Drive.kV);
        // Hardware
        this.driveMotor = new TalonFX(driveMotorId);
        this.turnMotor = new TalonFX(turnMotorId);
        this.turnEncoder = new DutyCycleEncoder(turnEncoderId, Constants.Swerve.FullRangeOffset, offset*Constants.Swerve.FullRangeOffset);
        
    }

    public Angle getTurnPosition() {
        double raw = this.turnEncoder.get(); // Reads the Raw data from the encoder
        double offset = Constants.Swerve.FullRangeOffset/2; // Adds possibility of negative values. For example, 90 is actuallly -90 and 270 is actually 90.
        return Units.Degrees.of(raw - offset);
    }

    public Rotation2d getRot2d() {
        return Rotation2d.fromRadians(this.getTurnPosition().in(Units.Radians));
    }

    // speed m/s
    public LinearVelocity getDriveSpeed() {
        // TODO
        // Translate the linear velocity read by the Kraken's encoder into a distance
        double angularVelocity = this.driveMotor.getVelocity().getValueAsDouble();
        return Units.MetersPerSecond.of(angularVelocity);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState.optimize(getRot2d());
        //setDriveSpeed( Units.MetersPerSecond.of(desiredState.speedMetersPerSecond) );
        setTurnPosition( Units.Radians.of(desiredState.angle.getRadians()) );
    }

    // m/s
    private void setDriveSpeed(LinearVelocity speed) {
        double metersPerSecond = speed.in(Units.MetersPerSecond);
        double volts = driveFeedforward.calculate(metersPerSecond) + drivePid.calculate(getDriveSpeed().in(Units.MetersPerSecond), metersPerSecond);
        this.driveMotor.setVoltage(volts);
    }

    private void setTurnPosition(Angle angle) {
        double setpoint = angle.in(Units.Radians);
        double measurement = getTurnPosition().in(Units.Radians);
        double volts = turnPid.calculate(measurement, setpoint) + turnFeedforward.calculate(measurement);
        this.turnMotor.setVoltage(-volts);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveMotor.getPosition().getValueAsDouble(), new Rotation2d(getTurnPosition()));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModule");
        builder.addDoubleProperty("angle/raw", turnEncoder::get, null);
        builder.addDoubleProperty("angle/measured", () -> getTurnPosition().in(Units.Radians), null);
        builder.addDoubleProperty("angle/setpoint", turnPid::getSetpoint, null);
        builder.addDoubleProperty("speed/raw", ()->driveMotor.getVelocity().getValueAsDouble(),null);
        builder.addDoubleProperty("speed/metersPerSecond", ()->getDriveSpeed().in(Units.MetersPerSecond), null);
        builder.addDoubleProperty("speed/setpoint", drivePid::getSetpoint, null);
        SmartDashboard.putData("%s-pid".formatted(this.getName()), turnPid);
    }
}
