package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The Intake for the robot:
 * 
 * <ul>
 *  <li> Must be completely vertical on match start
 *  <li> Must lower to [VALUE] on match start
 *  <li> 2 motors total, 1 for angle setting and 1 for spinning the flywheel
 *  <li> Constant values for intake
 * </ul>
 */

public class Intake extends SubsystemBase {

    private final SparkMax angleMotor = new SparkMax(Constants.Intake.INTAKE_ANGLE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig angleConfig = new SparkMaxConfig();

    private final SparkMax flywheelMotor = new SparkMax(Constants.Intake.INTAKE_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig flywheelConfig = new SparkMaxConfig();

    private double angle = Constants.Intake.STOWED_ANGLE;

    private final ProfiledPIDController anglePID = new ProfiledPIDController(
            Constants.Intake.AnglePID.kP,
            Constants.Intake.AnglePID.kI,
            Constants.Intake.AnglePID.kD,
            new TrapezoidProfile.Constraints(
                Constants.Intake.ANGLE_MAX_VELOCITY, 
                Constants.Intake.ANGLE_MAX_ACCELERATION
            )
        );
    private final ArmFeedforward angleFF = new ArmFeedforward(
            Constants.Intake.AngleFeedForward.kS,
            Constants.Intake.AngleFeedForward.kG,
            Constants.Intake.AngleFeedForward.kV
        );

    /**
     * Constructs a new intake
     */
    public Intake(){
        configureMotors();
        this.setDefaultCommand(stowIntakeCommand());
    }

    private void configureMotors(){
        // Angle Config
        angleConfig.softLimit.reverseSoftLimit(Constants.Intake.STOWED_ANGLE); // TODO
        angleConfig.softLimit.forwardSoftLimit(Constants.Intake.INTAKING_ANGLE);
        angleConfig.idleMode(IdleMode.kBrake);
        angleConfig.inverted(true);
        angleConfig.encoder.positionConversionFactor(Constants.Intake.POSITION_CONVERSION_FACTOR); // Converts the rotations to angle in radians      
        angleConfig.encoder.velocityConversionFactor(Constants.Intake.VELOCITY_CONVERSION_FACTOR); // Converts RPM to rad/s
        angleConfig.smartCurrentLimit(40);
        angleMotor.configure(angleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // Flywheel Config
        flywheelConfig.idleMode(IdleMode.kCoast);
        flywheelConfig.smartCurrentLimit(40);
        flywheelMotor.configure(flywheelConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public double getAngle(){
        return angleMotor.getEncoder().getPosition();
    }

    public void setAngle(double angle){
        angleMotor.setVoltage(
            anglePID.calculate(getAngle(), angle) +
            angleFF.calculate(getAngle(), 0)
        );
    }

    public void setAngleVoltage(double volts){
        angleMotor.setVoltage(volts);
    }

    public boolean canShoot(){
        return getAngle() >= Constants.Intake.SHOOT_THRESHOLD;
    }

    private void stopFlywheel(){
        flywheelMotor.set(0);
    }

    private void startFlywheel(){
        flywheelMotor.set(Constants.Intake.FLYWHEEL_SPEED);
    }

    public void lowerIntake(){
        setAngle(Constants.Intake.INTAKING_ANGLE);
    }

    public void raiseIntake(){
        setAngle(Constants.Intake.STOWED_ANGLE);
    }

    public Command runIntakeCommand(){
        return this.run(()->{
            lowerIntake();
            startFlywheel();
        });
    }

    public Command stowIntakeCommand(){
        return this.run(()->{
            raiseIntake();
            stopFlywheel();
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ShooterSubsystem");
        builder.addDoubleProperty("angle/measured", this::getAngle, null);
        SmartDashboard.putData("Intake/angle/pid", anglePID);
        SmartDashboard.putData("Intake/run", runIntakeCommand());
        SmartDashboard.putData("Intake/stow", stowIntakeCommand());
    }

}
