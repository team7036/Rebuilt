package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class PidControlContext implements ControlContext {
    private final PIDController drivePid, turnPid;
    private final SimpleMotorFeedforward driveFF;

    public PidControlContext() {
        this.drivePid = Constants.Swerve.Pid.DRIVE.createController();
        this.turnPid = Constants.Swerve.Pid.TURN.createController();

        this.driveFF = Constants.Swerve.Feedforward.DRIVE_PID.createFeedforward();
    }

    @Override
    public double calculate(double input, double state, double velocity, boolean turn) {
        if(turn) {
            return turnPid.calculate(state, input);
        }
        return drivePid.calculate(state, input)
                + driveFF.calculate(input);
    }

    @Override
    public void reset() {
        drivePid.reset();
        turnPid.reset();
    }
}
