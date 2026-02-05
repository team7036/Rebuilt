package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.subsystem.DefaultDrivetrainCommand;
import frc.robot.hardware.Hardware;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.messagingv1.MessageBus;
import frc.robot.util.messagingv1.MessageChannel;

public class RobotContainer {
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final Hardware hardware;

    private final Drivetrain drivetrain;

    private final MessageBus messageBus;

    public RobotContainer(Robot robot) {
        this.hardware = robot.getHardware();

        this.drivetrain = new Drivetrain(this);

        this.messageBus = new MessageBus(MessageBus.MessageExecutorType.SINGLE);

        configureBindings();
    }

    private void configureBindings() {

        //noinspection FunctionalExpressionCanBeFolded
        drivetrain.setDefaultCommand(
                new DefaultDrivetrainCommand(
                        drivetrain,
                        driverController::getLeftX,
                        driverController::getLeftY,
                        driverController::getRightX,
                        driverController.rightBumper()::getAsBoolean
                )
        );
    }

    public Command getAutonomousCommand() {
        return Autos.exampleAuto(exampleSubsystem);
    }

    public Hardware getHardware() {
        return this.hardware;
    }

    public MessageChannel createChannel(String channelId, MessageChannel.MessageReceiver receiver) {
        return new MessageChannel(
                this.messageBus,
                channelId,
                receiver
        );
    }
}
