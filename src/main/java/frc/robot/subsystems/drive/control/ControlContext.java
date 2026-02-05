package frc.robot.subsystems.drive.control;

public interface ControlContext {
    /**
     * Calculates voltage for motor given input
     * @param input speedMetersPerSecond for drive motors, angleRadians for turn motors
     * @param state current state motor is in
     * @param turn if the motor is a turn motor
     * @return Voltage for motor
     */
    double calculate(double input, double state, boolean turn);
    default void reset() {}
}
