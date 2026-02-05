package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class PidControlContext implements ControlContext {
    private final PIDController drivePid, turnPid;
    private final SimpleMotorFeedforward driveFF, turnFF;

    public PidControlContext() {
        this.drivePid = Constants.Swerve.Pid.DRIVE.createController();
        this.turnPid = Constants.Swerve.Pid.TURN.createController();

        this.driveFF = Constants.Swerve.Feedforward.DRIVE_PID.createFeedforward();
        this.turnFF = Constants.Swerve.Feedforward.TURN_PID.createFeedforward();
    }

    @Override
    public double calculate(double input, double state, boolean turn) {
        if(turn) {
            return turnPid.calculate(state, input)
                    + turnFF.calculate(input);
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
