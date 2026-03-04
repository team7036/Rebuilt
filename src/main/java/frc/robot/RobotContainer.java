package frc.robot;
import frc.robot.Constants.IDs;
import frc.robot.commands.subsystem.DefaultDrivetrainCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drivetrain;

public class RobotContainer {

    private final CommandXboxController driverController =
            new CommandXboxController(IDs.Controllers.Driver);
    private final Drivetrain drivetrain = new Drivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
                new DefaultDrivetrainCommand(
                        drivetrain,
                        this.driverController
                )
        );
        SmartDashboard.putData(drivetrain);
        //System.out.println(Arrays.toString(this.drivetrain.getEncoderValues()));
    }
}
