package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    private final SparkMax angleMotor, flywheelMotor;
    private final SparkMaxConfig angleConfig, flywheelConfig;
    private final ProfiledPIDController anglePID;
    private final ArmFeedforward angleFeedForward;

    /**
     * Constructs a new intake
     */
    public Intake() {
        this.angleMotor = new SparkMax(Constants.Intake.INTAKE_ANGLE_MOTOR_ID, MotorType.kBrushless);
        this.angleConfig = new SparkMaxConfig();
        this.flywheelMotor = new SparkMax(Constants.Intake.INTAKE_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        this.flywheelConfig = new SparkMaxConfig();
        this.anglePID = new ProfiledPIDController(
            Constants.Intake.AnglePID.kP,
            Constants.Intake.AnglePID.kI,
            Constants.Intake.AnglePID.kD,
            new TrapezoidProfile.Constraints(
                Constants.Intake.ANGLE_MAX_VELOCITY, 
                Constants.Intake.ANGLE_MAX_ACCELERATION
            )
        );

        this.angleFeedForward = new ArmFeedforward(
            Constants.Intake.AngleFeedForward.kS,
            Constants.Intake.AngleFeedForward.kG,
            Constants.Intake.AngleFeedForward.kV
        );
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

    public Command stowIntakeCommand(){
        return this.runOnce(()->flywheelMotor.set(0)).andThen(raiseIntakeCommand());
    }

    private double getAngle(){
        return angleMotor.getEncoder().getPosition();
    }

    private void setAngle(double angle){
        
        double pid = anglePID.calculate(getAngle(), angle);
        double ff = angleFeedForward.calculate(angle - Constants.Intake.OFFSET_ANGLE, 0);
        angleMotor.setVoltage(pid + ff);
    }

    private Command lowerIntakeCommand(){
        return this.run(()->setAngle(Constants.Intake.INTAKING_ANGLE)).until(anglePID::atSetpoint);
    }

    private Command raiseIntakeCommand(){
        return this.run(()->setAngle(Constants.Intake.STOWED_ANGLE));
    }

    public Command intakeFuelCommand(){
        return lowerIntakeCommand().andThen(()->flywheelMotor.set(Constants.Intake.FLYWHEEL_SPEED)).repeatedly();
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ShooterSubsystem");
        builder.addDoubleProperty("angle/measured", this::getAngle, null);
        builder.addDoubleProperty("angle/goal", ()->anglePID.getGoal().position, null);
        builder.addDoubleProperty("angle/positionError", anglePID::getPositionError, null);
        SmartDashboard.putData("Intake/AnglePID", anglePID);
        SmartDashboard.putData("Intake/raise", raiseIntakeCommand());
        SmartDashboard.putData("Intake/lower", lowerIntakeCommand());
    }

}
