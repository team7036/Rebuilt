package frc.robot.commands.subsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drivetrain;

import java.util.Arrays;
import java.util.function.Function;

import javax.xml.xpath.XPath;

public class DefaultDrivetrainCommand extends Command {
    private static final Function<Double, SlewRateLimiter> RATE_LIMITER_FACTORY = SlewRateLimiter::new;

    private final Drivetrain drivetrain;

    private final CommandXboxController driveController;

    private final SlewRateLimiter xSpeedLimiter = RATE_LIMITER_FACTORY.apply(3.0);
    private final SlewRateLimiter ySpeedLimiter = RATE_LIMITER_FACTORY.apply(3.0);
    private final SlewRateLimiter rotSpeedLimiter = RATE_LIMITER_FACTORY.apply(3.0);

    private final double maxSpeed, maxAngularSpeed;

    public DefaultDrivetrainCommand(Drivetrain drivetrain, CommandXboxController driveController) {
        this.drivetrain = drivetrain;
        this.maxSpeed = Constants.Drivetrain.MaxSpeed;
        this.maxAngularSpeed = Constants.Drivetrain.MaxAngularSpeed;

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
