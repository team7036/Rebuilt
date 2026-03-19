// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

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
        // Swerve Module Specs
        public static final AngularVelocity FREE_SPEED_RPM = Units.RPM.of(108); // rpm
        public static final Distance WHEEL_DIAMETER = Units.Meters.of(Units.Inches.of(4.0).in(Units.Meters)); // meters, 4 inches
        public static final Distance WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.times(Math.PI) ; // meters
        public static final double DRIVE_GEAR_RATIO = 6.12; 
        public static final LinearVelocity MAX_LINEAR_VELOCITY = FREE_SPEED_RPM.asFrequency().times(WHEEL_CIRCUMFERENCE);
         //public static double MaxAngularSpeed = MaxSpeed / (WHEEL_DIAMETER.in(Units.Meters) / 2); // rad/s
        
        public static final double MOTOR_ROTATIONS_PER_METER = WHEEL_CIRCUMFERENCE.in(Units.Meters) * Math.PI;
    }

    public final class Swerve {
       //public static double ConversionFactor = -360;

       public static double EncoderFullRange = 2*Math.PI;

        public static class EncoderOffset {
            public static double FrontLeft = 0.09462320236558006;
            public static double FrontRight = 0.4573415114335378;
            public static double BackLeft = 0.09562225239055631;
            public static double BackRight = 0.7770506444262661;
        }

        public static class Position {
            public static Translation2d FrontLeft = new Translation2d();
            public static Translation2d FrontRight = new Translation2d();
            public static Translation2d BackLeft = new Translation2d();
            public static Translation2d BackRight = new Translation2d();
        }

        public static class Feedforward {
            public static class Drive {
                public static double kS = 0;
                public static double kV = 1.3;
            }
            public static class Turn {
                public static double kS = 0;
                public static double kV = 0;
            }
        }
        public static class PID {
            public static class Drive {
                public static double kP = 3;
                public static double kI = 0;
                public static double kD = 0;
            }
            public static class Turn {
                public static double kP = 3
                ;
                public static double kI = 0;
                public static double kD = 0;
            }
        }
    }
}
