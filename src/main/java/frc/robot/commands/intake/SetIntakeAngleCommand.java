// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;



/** An example command that uses an example subsystem. */
public class SetIntakeAngleCommand extends Command
{
    private final Intake intake;
    private final double targetAngle;
    private final ProfiledPIDController pid = new ProfiledPIDController(
            Constants.Intake.AnglePID.kP,
            Constants.Intake.AnglePID.kI,
            Constants.Intake.AnglePID.kD,
            new TrapezoidProfile.Constraints(
                Constants.Intake.ANGLE_MAX_VELOCITY, 
                Constants.Intake.ANGLE_MAX_ACCELERATION
            )
        );
    private final ArmFeedforward ff = new ArmFeedforward(
            Constants.Intake.AngleFeedForward.kS,
            Constants.Intake.AngleFeedForward.kG,
            Constants.Intake.AngleFeedForward.kV
        );
    
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SetIntakeAngleCommand(Intake intake, double targetAngle)
    {
        this.intake = intake;
        this.targetAngle = targetAngle;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
    }
    
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.setAngleVoltage(
            pid.calculate(intake.getAngle(), targetAngle) +
            ff.calculate(intake.getAngle(), 0)
        );
    }
    
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
    
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return pid.atSetpoint();
    }
}
