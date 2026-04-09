package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drivetrain;

public class DefaultDrivetrainCommand extends Command {
    private final Drivetrain drivetrain;

    private final CommandXboxController driveController;

    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotSpeedLimiter = new SlewRateLimiter(3);

    private final double maxSpeed = Constants.Drivetrain.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond);
    private final double maxAngularSpeed = Constants.Drivetrain.MAX_ANGULAR_VELOCITY.in(Units.RadiansPerSecond);
    public DefaultDrivetrainCommand(Drivetrain drivetrain, CommandXboxController driveController) {
        this.drivetrain = drivetrain;

        this.driveController = driveController;

        this.setName("DefaultDTCommand");

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double leftX = this.driveController.getLeftX();
        double leftY = this.driveController.getLeftY();
        double rightX = this.driveController.getRightX();

        boolean half = this.driveController.rightBumper().getAsBoolean();

        double xSpeed = (half ? maxSpeed / 2 : maxSpeed)
                * xSpeedLimiter.calculate(MathUtil.applyDeadband(leftX, 0.04));
        double ySpeed = (half ? maxSpeed / 2 : maxSpeed)
                * -ySpeedLimiter.calculate(MathUtil.applyDeadband(leftY, 0.04));
        double rot = (half ? maxAngularSpeed / 2 : maxAngularSpeed)
                * -rotSpeedLimiter.calculate(MathUtil.applyDeadband(rightX, 0.04));

        this.drivetrain.driveRobotRelative(new ChassisSpeeds(xSpeed, ySpeed, rot));
    }
}
