package frc.robot.subsystems.drive;

import java.util.function.BiFunction;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
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
    /* WHAT BI-FUNCTIONS ARE
    * BiFunction<I1, I2, R1>
    * I1 - Type of input 1 (For example, Double or String or any class)
    * I2 - Type of input 2 (For example, Double or String or any class)
    * R1 - Type of return value (For example, Double or String or any class)
    * 
    * EXAMPLE:
    * 
    * BiFunction<String, Integer, String> ADD_TO_END = (string, num) -> string+num;
    * 
    * Then, to call it:
    * ADD_TO_END.apply("some string!", 53);
    * 
    * REMEMBER, apply requires (I1 input1, I2 input2)
    * In this case, the apply requires I1 to be a string, which is "some string!"
    * Similarly, I2 must be an integer, in this case 53
    * 
    * Which will output:
    * "some string!53"
    *
    */
    public static BiFunction<Double, Double, Double> TO_DEGREES_FROM_RAW = (raw, offset) -> {
        return ((raw * 360) + (360 - offset)) % 360; //Raw -> Degrees -> positive offset -> wrap degrees
    };

    private final PIDController turnPid, drivePid;
    private final SimpleMotorFeedforward driveFeedforward, turnFeedforward;

    private final TalonFX driveMotor, turnMotor;
    private final DutyCycleEncoder turnEncoder;
    private final Angle offset;

    public SwerveModule(String moduleName, int driveMotorId, int turnMotorId, int turnEncoderId, Angle offset) {
        super("SM-%s".formatted(moduleName));
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
        this.turnEncoder = new DutyCycleEncoder(turnEncoderId);

        this.offset = offset;
    }

    // Radians
    public Angle getTurnPosition() {
        double offsetDeg = this.offset.magnitude();
        return Units.Degrees.of(TO_DEGREES_FROM_RAW.apply(this.turnEncoder.get(), offsetDeg));
    }

    public Rotation2d getRot2d() {
        return Rotation2d.fromRadians(this.getTurnPosition().in(Units.Radians));
    }

    // speed m/s
    public LinearVelocity getDriveSpeed() {
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
        double setpoint = MathUtil.angleModulus(angle.in(Units.Radians));
        double measurement = getTurnPosition().in(Units.Radians);
        double volts = turnPid.calculate(measurement, setpoint) + turnFeedforward.calculate(measurement);
        //this.turnMotor.setVoltage(volts);
        this.turnMotor.setVoltage(1);
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
