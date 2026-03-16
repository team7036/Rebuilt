package frc.robot;

import frc.robot.Constants.Controllers;
import frc.robot.commands.subsystem.DefaultDrivetrainCommand;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.units.Units;

public class RobotContainer {

    private final CommandXboxController driverController =
            new CommandXboxController(Controllers.DRIVER_PORT);

    private final Shooter shooter = new Shooter();

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

        // just to test that this command works when we introduce the XBoxController
        driverController.a().whileTrue(shooter.fireCommand(AngularVelocity.ofBaseUnits(2270, Units.RPM), true)); 
    }   
}
