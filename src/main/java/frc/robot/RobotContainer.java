package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drivetrain;

public class RobotContainer {

    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);


    private final Drivetrain drivetrain = new Drivetrain();
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

        //System.out.println(Arrays.toString(this.drivetrain.getEncoderValues()));

        this.drivetrain.testModules();
    }
}
