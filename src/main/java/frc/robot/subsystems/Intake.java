package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
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
    private final SparkMax intakeMotor, angleMotor;
    private final RelativeEncoder intakeAngleEncoder;

    private final ProfiledPIDController intakeAnglePID;
    private final ArmFeedforward intakeAngleFF;

    /**
     * Constructs a new intake
     */
    public Intake() {
        this.setDefaultCommand(setStowedCommand());
        this.angleMotor = new SparkMax(Constants.Intake.INTAKE_ANGLE_MOTOR_ID, MotorType.kBrushless);
        this.intakeMotor = new SparkMax(Constants.Intake.INTAKE_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        this.intakeAngleEncoder = this.angleMotor.getEncoder();

        Constraints constraints = new Constraints(
            Constants.Intake.INTAKE_MAX_VELOCITY,
            Constants.Intake.INTAKE_MAX_ACCELERATION); 

        this.intakeAnglePID = new ProfiledPIDController(
            Constants.Intake.PID.kP,
            Constants.Intake.PID.kI,
            Constants.Intake.PID.kD,
            constraints
        );
        this.intakeAngleFF = new ArmFeedforward(
            Constants.Intake.ArmFeedforward.kS,
            Constants.Intake.ArmFeedforward.kG,
            Constants.Intake.ArmFeedforward.kV
        );
    }

    /**
     * Retrieves the angle of the intake arm in radians
     * @return radians of the intake arm
     */
    private double getIntakeAngle() {
        return this.intakeAngleEncoder.getPosition();
    }

    /**
     * Sets the angle of the intake arm to a given angle
     * @param angle The angle to set the arm to
     * @implNote The angle passed in can be in any angle unit as it is converted to radians.
     */
    private void setIntakeAngle(Angle angle) {
        double rads = angle.in(Units.Radians);
        this.angleMotor.setVoltage(
            -(this.intakeAngleFF.calculate(this.getIntakeAngle(), 0) +
            this.intakeAnglePID.calculate(this.getIntakeAngle(), rads))
        );
    }
    /**
     * Sets the arm of the intake to the pre-match angle
     * @return The command to set the arm to the pre-match angle
     */
    //TODO Find angle for pre-match angle instead of 0 degrees
    // public Command setPrematchAngle() {
    //     return this.run(() -> this.setIntakeAngle(
    //         Units.Degrees.of(0)
    //     )).until(this.intakeAnglePID::atSetpoint);
    // }

    /**
     * Sets the arm of the intake to the resting angle
     * @return The command to set the arm to the resting angle
     */
    //TODO Find angle for resting angle instead of 90 degrees
    public Command setStowedCommand() {
        return this.runOnce(()->this.intakeMotor.set(0)).andThen(
            this.run(() -> this.setIntakeAngle(
                Constants.Intake.STOWED_ANGLE
            )).until(this.intakeAnglePID::atSetpoint)
        );
    }

    /**
     * Sets the arm of the intake to the intaking angle
     * @return The command to set the arm to the intaking angle
     */
    //TODO Find angle for intaking angle instead of 85 degrees
    public Command setDeployedCommand() {
        return this.runOnce(()->this.intakeMotor.set(0)).andThen(
            this.run(() -> this.setIntakeAngle(
                Constants.Intake.DEPLOYED_ANGLE
            )).until(this.intakeAnglePID::atSetpoint)
        );
    }

    /**
     * Startups the flywheel to allow for intaking
     * @return The command to startup the flywheel
     */
    //TODO Find volts for intake
    public Command startIntakeCommand() {
        return this.run(() -> this.intakeMotor.setVoltage(Constants.Intake.INTAKE_MOTOR_VOLTAGE));
    }
    /**
     * Shuts down the flywheel
     * @return The command to kill the flywheel
     * @implNote Sets the voltage of the flywheel to 0
     */
    public Command killFlywheel() {
        return this.run(() -> this.intakeMotor.setVoltage(
            Units.Volts.of(0)
        ));
    }
}
