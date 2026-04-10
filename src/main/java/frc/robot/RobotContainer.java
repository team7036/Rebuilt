package frc.robot;
import frc.robot.Constants.IDs;
import frc.robot.commands.DefaultDrivetrainCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drivetrain;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(IDs.Controllers.Driver);
    private final CommandXboxController operatorController = new CommandXboxController(IDs.Controllers.Operator);
    private final Drivetrain drivetrain = new Drivetrain();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Vision vision = new Vision();
    public RobotContainer() {
        configureDashboard();
        configureBindings();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            new DefaultDrivetrainCommand(
                drivetrain,
                this.driverController
            )
        );

        operatorController
            .rightBumper()
            .whileTrue(shooter.fireFuelCommand().alongWith(intake.runIntakeCommand()));
        
        operatorController
            .a()
            .onTrue(intake.runIntakeCommand().alongWith(shooter.intakeFuelCommand()));
        
        operatorController
            .y()
            .onTrue(intake.stowIntakeCommand());

    }

    private void configureDashboard(){
        SmartDashboard.putData(drivetrain);
        SmartDashboard.putData(shooter);
        SmartDashboard.putData(intake);
        SmartDashboard.putData(vision);
    }
}
