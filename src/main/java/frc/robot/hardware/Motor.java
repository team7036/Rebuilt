package frc.robot.hardware;

public interface Motor extends HardwareDevice {
    /**
     * Sets the voltage of the motor
     * @param volts voltage to set the motor to
     */
    void setVoltage(double volts);

    /**
     * Retrieves the motor's max speed in m/s
     * @return max speed in m/s
     */
    double getMaxSpeed();

    /**
     * Retrieves the encoder assigned to this motor
     * @return this motor's encoder
     */
    Encoder getEncoder();
}
