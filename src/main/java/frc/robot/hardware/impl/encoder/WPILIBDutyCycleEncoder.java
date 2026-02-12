package frc.robot.hardware.impl.encoder;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.Hardware;
import frc.robot.util.periodicsv1.Periodic;
import frc.robot.util.periodicsv1.PeriodicManager;

public class WPILIBDutyCycleEncoder implements AbsoluteEncoder, Periodic {
    private static int ID = 0;

    private final DutyCycleEncoder encoder;
    private final LinearFilter velocityFilter;

    private double lastTime;
    private double lastAngle;

    private double curAngularVel;

    private double accumulatedAngle = 0.0;
    private double lastRawAngle = 0.0;
    private boolean initialized = false;

    private double offsetRadians = 0.0;


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
        double raw = encoder.get() * 2.0 * Math.PI;

        if (!initialized) {
            lastRawAngle = raw;
            initialized = true;
        }

        double delta = MathUtil.angleModulus(raw - lastRawAngle);
        accumulatedAngle += delta;

        lastRawAngle = raw;

        return MathUtil.angleModulus(accumulatedAngle - offsetRadians);
    }

    @Override
    public double getVelocity() {
        return curAngularVel;
    }

    @Override
    public void setOffset(double offsetRadians) {
        this.offsetRadians = offsetRadians;
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

        if (dT > 1e-5) {
            curAngularVel = velocityFilter.calculate(dA / dT);
        }

        lastAngle = curAngle;
        lastTime = curTime;
    }
}
