package frc.robot.hardware;

public interface Encoder extends HardwareDevice {
    /**
     * Get the position of the wheels
     * @return wheel position in meters for drive, radians for turn
     */
    double getPosition();
    /**
     * Get the velocity of the wheels
     * @return wheel speed in meters per second for drive, radians per second for turn
     */
    double getVelocity();
}
