package frc.robot.custom;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Encoder;

public class TalonFXEncoder extends Encoder {
    private final TalonFX motor;
    private double distancePerPulse = 1.0;
    private boolean reverseDirection = false;
    
    /**
     * Construct a TalonFXEncoder using the integrated encoder
     * @param motor The TalonFX motor to use
     */
    public TalonFXEncoder(TalonFX motor) {
        // Pass dummy values to parent constructor to avoid hardware initialization
        // The parent class will try to create a counter, but we'll override all methods
        super(-1, -1); // Use invalid channels that won't conflict
        this.motor = motor;
    }
    
    @Override
    public int get() {
        double position = motor.getPosition().getValueAsDouble();
        return (int) (reverseDirection ? -position : position);
    }
    
    @Override
    public void reset() {
        motor.setPosition(0.0);
    }
    
    @Override
    public double getPeriod() {
        double velocity = motor.getVelocity().getValueAsDouble();
        if (velocity == 0) return Double.POSITIVE_INFINITY;
        return 1.0 / Math.abs(velocity);
    }
    
    @Override
    public void setDistancePerPulse(double distancePerPulse) {
        this.distancePerPulse = distancePerPulse;
    }
    
    @Override
    public double getDistancePerPulse() {
        return distancePerPulse;
    }
    
    @Override
    public double getDistance() {
        return get() * distancePerPulse;
    }
    
    @Override
    public double getRate() {
        double velocity = motor.getVelocity().getValueAsDouble();
        return (reverseDirection ? -velocity : velocity) * distancePerPulse;
    }
    
    @Override
    public void setReverseDirection(boolean reverseDirection) {
        this.reverseDirection = reverseDirection;
    }
    
    @Override
    public boolean getDirection() {
        double velocity = motor.getVelocity().getValueAsDouble();
        return reverseDirection ? velocity < 0 : velocity > 0;
    }
    
    @Override
    public boolean getStopped() {
        return Math.abs(motor.getVelocity().getValueAsDouble()) < 1e-6;
    }
}