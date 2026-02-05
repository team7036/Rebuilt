package frc.robot.hardware.impl.encoder;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.hardware.Hardware;
import frc.robot.util.periodicsv1.Periodic;
import frc.robot.util.periodicsv1.PeriodicManager;

public class WPILIBDutyCycleEncoder implements frc.robot.hardware.Encoder, Periodic {
    private static int ID = 0;

    private final DutyCycleEncoder encoder;
    private final LinearFilter velocityFilter;

    private double lastTime;
    private double lastAngle;

    private double curAngularVel;

    public WPILIBDutyCycleEncoder(DutyCycleEncoder encoder) {
        this.encoder = encoder;
        this.velocityFilter = LinearFilter.backwardFiniteDifference(1, 3, 0.02);

        this.lastTime = Timer.getFPGATimestamp();
        this.lastAngle = this.getPosition();

        PeriodicManager.registerPeriodic("dce-" + ID, this);
        ID++;
    }

    @Override
    public double getPosition() {
        return encoder.get() * Math.PI * 2; //Percentage -> radians
    }

    @Override
    public double getVelocity() {
        return curAngularVel;
    }

    public static WPILIBDutyCycleEncoder of(Hardware hardware, int dioPort) {
        return new WPILIBDutyCycleEncoder(
                new DutyCycleEncoder(dioPort)
        );
    }

    @Override
    public void periodic() {
        double curTime = Timer.getFPGATimestamp();
        double curAngle = this.getPosition();

        double dA = MathUtil.angleModulus(curAngle - lastAngle); //deltaAngle
        double dT = curTime - lastTime; //deltaTime

        curAngularVel = velocityFilter.calculate(dA/dT);

        lastAngle = curAngle;
        lastTime = curTime;
    }
}
