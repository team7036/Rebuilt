package frc.robot.hardware;

public interface Gyro extends HardwareDevice {
    /**
     * Gets the angle of the gyro
     * @return angle in radians
     */
    double getAngle();
    /**
     * Resets the angle of the gyro to 0
     */
    void reset();
}
