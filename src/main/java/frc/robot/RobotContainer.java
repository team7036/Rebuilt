package frc.robot;
import frc.robot.Constants.IDs;
import frc.robot.commands.subsystem.DefaultDrivetrainCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drivetrain;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(IDs.Controllers.Driver);
    private final CommandXboxController operatorController = new CommandXboxController(IDs.Controllers.Operator);
    private final Drivetrain drivetrain = new Drivetrain();
    private final Shooter shooter = new Shooter();
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

        operatorController.a().whileTrue(shooter.fireFuelCommand());
    }

    private void configureDashboard(){
        SmartDashboard.putData(drivetrain);
        SmartDashboard.putData(shooter);
    }
}
