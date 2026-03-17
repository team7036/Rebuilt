package frc.robot;

import frc.robot.Constants.Controllers;
import frc.robot.subsystems.Storage;
import frc.robot.commands.subsystem.DefaultDrivetrainCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drivetrain;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(Controllers.DRIVER_PORT);

    private final Storage storage = new Storage();

    // private final Drivetrain drivetrain = new Drivetrain();
    // public RobotContainer() {
    // configureBindings();
    // }

    private void configureBindings() {
        driverController.y().whileTrue(storage.startFeedCommand());

        // drivetrain.setDefaultCommand(
        // new DefaultDrivetrainCommand(
        // drivetrain,
        // this.driverController
        // )
        // );
    }

}
