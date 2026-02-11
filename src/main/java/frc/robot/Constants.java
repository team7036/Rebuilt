// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.hardware.Encoder;
import frc.robot.hardware.Hardware;
import frc.robot.hardware.Motor;
import frc.robot.util.LazyCachedFunction;
import frc.robot.util.LazyValue;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public final class Constants {

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    /**
     * {@code Swerve} contains everything related to swerve drive.
     */
    public static class Swerve {
        // Selects which control type the swerve modules will use
        // For now, either PID (which includes a Feedforward) or LQR
        public static final SwerveControllerType CONTROLLER_TYPE =
                SwerveControllerType.PID;

        public static final boolean USE_AUTO = false;

        public static final RobotConfig PATHPLANNER_CONFIG;

        static {
            RobotConfig config;
            try {
                config = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                if(USE_AUTO) throw new RuntimeException("Failed to get pathplanner config!", e);
                else config = null;
            }

            PATHPLANNER_CONFIG = config;
        }

        /**
         * {@code Pid} contains everything related to PID constants of swerve modules
         */
        public static class Pid {
            public static final double I_RANGE = 0.15;

            public static final PIDConstants DRIVE =
                    PIDConstants.of(0, 0, 0, false);
            public static final PIDConstants TURN =
                    PIDConstants.of(4.4, 0, 0.1, true);
        }

        /**
         * {@code Feedforward} contains everything related to FF constants of swerve modules
         */
        public static class Feedforward {
            public static final FFConstants DRIVE_PID =
                    FFConstants.of(0, 1.3);
        }

        /**
         * {@code LQR} contains everything related to LQR constants of swerve modules
         */
        public static class LQR {
            public static final DriveLQRConstants DRIVE =
                    DriveLQRConstants.of(2.0, 0.3);

            public static final TurnLQRConstants TURN =
                    TurnLQRConstants.of(1.0, 0.1);

            public static final double DT = 0.02;
        }

        /**
         * {@code Hardware} contains everything related to constructing the hardware classes
         * of swerve modules
         */
        public static class Hardware {
            public static final SwerveModuleConstants FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT;
            public static final Translation2d FL_POS, FR_POS, BL_POS, BR_POS;
            static {
                //Todo find offsets for turn encoders
                FRONT_LEFT = SwerveModuleConstants.of(
                        10,
                        11,
                        0,
                        0
                );
                FRONT_RIGHT = SwerveModuleConstants.of(
                        13,
                        14,
                        1,
                        0
                );
                BACK_LEFT = SwerveModuleConstants.of(
                        15,
                        16,
                        3,
                        0
                );
                BACK_RIGHT = SwerveModuleConstants.of(
                        17,
                        18,
                        4,
                        0
                );
                FL_POS = new Translation2d(
                        0.1715,
                        0.1715
                );
                FR_POS = new Translation2d(
                        0.1715,
                        -0.1715
                );
                BL_POS = new Translation2d(
                        -0.1715,
                        0.1715
                );
                BR_POS = new Translation2d(
                        -0.1715,
                        -0.1715
                );
            }
        }
    }



    public record PIDConstants(double p, double i, double d, LazyValue<PIDController> controllerLazy) {
        public PIDController createController() {
            return this.controllerLazy.get();
        }

        public com.pathplanner.lib.config.PIDConstants toPathPlannerConstants() {
            return new com.pathplanner.lib.config.PIDConstants(
                    this.p, this.i, this.d, Swerve.Pid.I_RANGE
            );
        }

        public static PIDConstants of(double p, double i, double d, boolean continuousPie) {
            return new PIDConstants(
                    p, i, d,
                    new LazyValue<>(
                            () ->{
                                PIDController pid = new PIDController(p, i, d);
                                if(continuousPie) pid.enableContinuousInput(-Math.PI, Math.PI);
                                return pid;
                            }
                    )
            );
        }
    }

    public record FFConstants(double s, double v, LazyValue<SimpleMotorFeedforward> ffLazy) {
        public SimpleMotorFeedforward createFeedforward() {
            return this.ffLazy.get();
        }

        public static FFConstants of(double s, double v) {
            return new FFConstants(
                    s, v,
                    new LazyValue<>(() -> new SimpleMotorFeedforward(s, v))
            );
        }
    }

    public record DriveLQRConstants(double v, double a, LazyValue<LinearSystem<N1, N1, N1>> lqrLazy) {
        public LinearSystem<N1, N1, N1> createSystem() {
            return this.lqrLazy.get();
        }

        public static DriveLQRConstants of(double v, double a) {
            return new DriveLQRConstants(
                    v, a,
                    new LazyValue<>(() -> LinearSystemId
                            .identifyVelocitySystem(v, a))
            );
        }
    }
    public record TurnLQRConstants(double v, double a, LazyValue<LinearSystem<N2, N1, N2>> lqrLazy) {
        public LinearSystem<N2, N1, N2> createSystem() {
            return this.lqrLazy.get();
        }

        public static TurnLQRConstants of(double v, double a) {
            return new TurnLQRConstants(
                    v, a,
                    new LazyValue<>(() -> LinearSystemId.identifyPositionSystem(v, a))
            );
        }
    }

    public record SwerveModuleConstants(int driveMotorID, int turnMotorID, int turnEncoderID, double turnEncoderOffset, LazyCachedFunction<Hardware, SwerveModuleHardware> hardwareLazy) {

        public SwerveModuleHardware createHardware(Hardware hardware) {
            return hardwareLazy.get(hardware);
        }

        public static SwerveModuleConstants of(int driveMotorID, int turnMotorID, int turnEncoderID, double turnEncoderOffset) {
            return new SwerveModuleConstants(
                    driveMotorID, turnMotorID, turnEncoderID, turnEncoderOffset,
                    new LazyCachedFunction<>((hardware) -> new SwerveModuleHardware(
                            hardware.createMotor(TalonFX.class, driveMotorID),
                            hardware.createMotor(TalonFX.class, turnMotorID),
                            hardware.createEncoder(DutyCycleEncoder.class, turnEncoderID)
                    ))
            );
        }
    }

    public record SwerveModuleHardware(Motor driveMotor, Motor turnMotor, Encoder turnEncoder) {}

    public enum SwerveControllerType {
        PID,
        LQR
    }
}
