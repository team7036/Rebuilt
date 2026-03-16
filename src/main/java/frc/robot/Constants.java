// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

public final class Constants {

    public final class Drivetrain {
        public static double maxSpeed = 1.0;
    }

    public final class Swerve {

        public static final Swerve.IDs FrontLeft = new Swerve.IDs(0, 0, 0, 0);
        public static final Swerve.IDs FrontRight = new Swerve.IDs(0, 0, 0, 0);
        public static final Swerve.IDs BackLeft = new Swerve.IDs(0, 0, 0, 0);
        public static final Swerve.IDs BackRight = new Swerve.IDs(0, 0, 0, 0);

        public static class IDs {
            public final int turnMotorId;
            public final int turnEncoderId;
            public final int driveMotorId;
            public final int driveEncoderId;
            public IDs(int turnMotorId, int turnEncoderId, int driveMotorId, int driveEncoderId){
                this.turnMotorId = turnMotorId;
                this.turnEncoderId = turnEncoderId;
                this.driveMotorId = driveMotorId;
                this.driveEncoderId = driveEncoderId;
            }
        }

        public static class Position {
            public static Translation2d FrontLeft = new Translation2d();
            public static Translation2d FrontRight = new Translation2d();
            public static Translation2d BackLeft = new Translation2d();
            public static Translation2d BackRight = new Translation2d();
        }

        public static class Control {
            public static class DriveFeedforward {
                public static int kS = 0;
                public static int kV = 0;
            }
            public static class TurnPID {
                public static int kP = 0;
                public static int kI = 0;
                public static int kD = 0;
            }
        }
    }

    /** Constants for the Vision subsystem */
    public static class Vision {
        // HEARTBEAT_LENIENCY - How many checks (peroidic, 20 ms) it can fail before setting Vision.active to false
        public static final int HEARTBEAT_LENIENCY = 3;
    }

    /** Constants for the Intake subsystem */
    public final class Intake {
        public static int INTAKE_ANGLE_MOTOR_ID = 40;
        public static int INTAKE_FLYWHEEL_MOTOR_ID = 55;
        public static double INTAKE_MOTOR_VOLTAGE = 5.0;

        private static double RAW_STOWED_ANGLE = 0.0;
        public static Angle STOWED_ANGLE = Angle.ofBaseUnits(RAW_STOWED_ANGLE, null);

        private static double RAW_DEPLOYED_ANGLE = -1.557;
        public static Angle DEPLOYED_ANGLE = Angle.ofBaseUnits(RAW_DEPLOYED_ANGLE, null);
        /*
         * ~30 (must be under at least, cannot be over) inches off ground
         * Robot is ~2 inches off ground
         * 
         * 20:1 specifically for motor
         * 1:1.575 for chain
         * Final:
         * ~10.33:1
         * 
         * Raw conversion factor:
         * 0.02930000051856041
         * 
         * Conversion accounting for gear ratio:
         * ~ 0.3026690054
         * 
         * Try 34.85 first
         */
        public static double INTAKE_MAX_VELOCITY = 0;
        public static double INTAKE_MAX_ACCELERATION = 0;

        public static class PID {
            public static double kP = 0;
            public static double kI = 0;
            public static double kD = 0;
        }

        public static class ArmFeedforward {
            public static double kS = 0;
            public static double kG = 0;
            public static double kV = 0;
        }
    }

    public static class Controllers {
        public static final int DRIVER_PORT = 0;
    }
}
