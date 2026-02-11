package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import frc.robot.Constants;

public class LQRControlContext implements ControlContext {
    private final LinearSystemLoop<N1, N1, N1> driveLoop;
    private final LinearSystemLoop<N2, N1, N2> turnLoop;

    public LQRControlContext() {
        LinearSystem<N1, N1, N1> drivePlant = Constants.Swerve.LQR.DRIVE.createSystem();

        LinearQuadraticRegulator<N1, N1, N1> driveLqr =
                new LinearQuadraticRegulator<>(
                        drivePlant,
                        VecBuilder.fill(1.0),
                        VecBuilder.fill(15.0),
                        Constants.Swerve.LQR.DT
                );

        KalmanFilter<N1, N1, N1> driveObserver =
                new KalmanFilter<>(
                        Nat.N1(),
                        Nat.N1(),
                        drivePlant,
                        VecBuilder.fill(0.1),
                        VecBuilder.fill(0.01),
                        Constants.Swerve.LQR.DT
                );

        driveLoop =
                new LinearSystemLoop<>(
                        drivePlant,
                        driveLqr,
                        driveObserver,
                        12.0,
                        Constants.Swerve.LQR.DT
                );

        LinearSystem<N2, N1, N2> turnPlant = Constants.Swerve.LQR.TURN.createSystem();

        LinearQuadraticRegulator<N2, N1, N2> turnLqr =
                new LinearQuadraticRegulator<>(
                        turnPlant,
                        VecBuilder.fill(1.5, 0.2),   // Q: [pos, vel] penalties
                        VecBuilder.fill(12.0),       // R: voltage penalty
                        Constants.Swerve.LQR.DT
                );

        KalmanFilter<N2, N1, N2> turnObserver =
                new KalmanFilter<>(
                        Nat.N2(),    // states
                        Nat.N2(),    // outputs (position AND velocity)
                        turnPlant,
                        VecBuilder.fill(0.1, 1.0),   // model noise for [pos, vel]
                        VecBuilder.fill(0.02, 0.5),  // measurement noise for [pos, vel]
                        Constants.Swerve.LQR.DT
                );

        turnLoop =
                new LinearSystemLoop<>(
                        turnPlant,
                        turnLqr,
                        turnObserver,
                        12.0,
                        Constants.Swerve.LQR.DT
                );
    }

    @Override
    public double calculate(double input, double state, double velocity, boolean isTurn) {
        if (isTurn) {
            turnLoop.setNextR(VecBuilder.fill(input, 0.0));

            turnLoop.correct(VecBuilder.fill(state, velocity));

            turnLoop.predict(Constants.Swerve.LQR.DT);

            return Math.max(-12, Math.min(turnLoop.getU(0), 12));
        } else {
            driveLoop.setNextR(VecBuilder.fill(input));
            driveLoop.correct(VecBuilder.fill(state));
            driveLoop.predict(Constants.Swerve.LQR.DT);

            return Math.max(-12, Math.min(driveLoop.getU(0), 12));
        }
    }



    @Override
    public void reset() {
        driveLoop.reset(VecBuilder.fill(0.0));
        turnLoop.reset(VecBuilder.fill(0.0, 0.0));
    }
}
