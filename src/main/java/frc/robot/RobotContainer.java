package frc.robot;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.Controllers;
// import frc.robot.commands.subsystem.DefaultDrivetrainCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.drive.Drivetrain;

public class RobotContainer {

    private final CommandXboxController driverController =
            new CommandXboxController(Controllers.DRIVER_PORT);
    private final Intake intake = new Intake();


    // private final Drivetrain drivetrain = new Drivetrain();
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        // drivetrain.setDefaultCommand(
        //         new DefaultDrivetrainCommand(
        //                 drivetrain,
        //                 this.driverController
        //         )
        // );
        

        driverController.b().toggleOnTrue(intake.startupFlywheel());

        //System.out.println(Arrays.toString(this.drivetrain.getEncoderValues()));
    }
}
