// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

    public final class IDs {

        public final class Controllers {
            public static int Driver = 0;
        }

        public final class DIO {
            public static int FrontLeftEncoder = 0;
            public static int FrontRightEncoder = 1;
            public static int BackLeftEncoder = 3;
            public static int BackRightEncoder = 4;

        }

        public final class CAN {
            public static int FrontLeftDrive = 10;
            public static int FrontLeftTurn = 11;
            public static int FrontRightDrive = 12;
            public static int FrontRightTurn = 13;
            public static int BackLeftDrive = 14;
            public static int BackLeftTurn = 15;
            public static int BackRightDrive = 16;
            public static int BackRightTurn = 17;
        }
    }

    public final class Drivetrain {

        public static double MaxSpeed = 1.0;
        public static double MaxAngularSpeed = MaxSpeed / 3.14;


        //Swerve Module Specs
        public static final double FREE_SPEED_RPM = 6000.0; //rpm
        public static final double FREE_SPEED_RPS = FREE_SPEED_RPM / 60.0; // 100 rps

        public static final double WHEEL_DIAMETER_METERS = 0.1016; //meters, 4 inches
        public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS; //meters
        public static final double DRIVE_GEAR_RATIO = 6.12;

        public static final double MOTOR_ROTATIONS_PER_METER =
            DRIVE_GEAR_RATIO / WHEEL_CIRCUMFERENCE_METERS;
    }

    public final class Swerve {

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
}
