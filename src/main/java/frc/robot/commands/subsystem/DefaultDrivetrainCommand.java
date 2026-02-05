package frc.robot.commands.subsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class DefaultDrivetrainCommand extends Command {

    private final Drivetrain drivetrain;

    private final DoubleSupplier leftX, leftY, rightX;
    private final BooleanSupplier half;

    public DefaultDrivetrainCommand(
            Drivetrain drivetrain,
            DoubleSupplier leftX,
            DoubleSupplier leftY,
            DoubleSupplier rightX,
            BooleanSupplier half
    ) {
        this.drivetrain = drivetrain;

        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;

        this.half = half;

        this.setName("DefaultDTCommand");

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3.0);
        SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3.0);
        SlewRateLimiter zSpeedLimiter = new SlewRateLimiter(3.0);

        double maxSpeed = this.drivetrain.maxSpeed();

        double xSpeed = (half.getAsBoolean() ? maxSpeed / 2 : maxSpeed)
                * xSpeedLimiter.calculate(MathUtil.applyDeadband(leftX.getAsDouble(), 0.04));
        double ySpeed = (half.getAsBoolean() ? maxSpeed / 2 : maxSpeed)
                * -ySpeedLimiter.calculate(MathUtil.applyDeadband(leftY.getAsDouble(), 0.04));
        double rot = (half.getAsBoolean() ? (double) 20 / 2 : 20)
                * -zSpeedLimiter.calculate(MathUtil.applyDeadband(rightX.getAsDouble(), 0.04));

        this.drivetrain.drive(new ChassisSpeeds(xSpeed, ySpeed, rot));

    }
}
