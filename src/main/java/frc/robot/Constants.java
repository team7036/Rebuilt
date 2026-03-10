// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BiFunction;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

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

        // Swerve Module Specs
        public static final double FREE_SPEED_RPM = 6000.0; // rpm
        public static final double FREE_SPEED_RPS = FREE_SPEED_RPM / 60.0; // 100 rps

        public static final double WHEEL_DIAMETER_METERS = 0.1016; // meters, 4 inches
        public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS; // meters
        public static final double DRIVE_GEAR_RATIO = 6.12;

        public static final double MOTOR_ROTATIONS_PER_METER = DRIVE_GEAR_RATIO / WHEEL_CIRCUMFERENCE_METERS;
    }

    public final class Swerve {
        
        /* WHAT BI-FUNCTIONS ARE
         * BiFunction<I1, I2, R1>
         * I1 - Type of input 1 (For example, Double or String or any class)
         * I2 - Type of input 2 (For example, Double or String or any class)
         * R1 - Type of return value (For example, Double or String or any class)
         * 
         * EXAMPLE:
         * 
         * BiFunction<String, Integer, String> ADD_TO_END = (string, num) -> string+num;
         * 
         * Then, to call it:
         * ADD_TO_END.apply("some string!", 53);
         * 
         * REMEMBER, apply requires (I1 input1, I2 input2)
         * In this case, the apply requires I1 to be a string, which is "some string!"
         * Similarly, I2 must be an integer, in this case 53
         * 
         * Which will output:
         * "some string!53"
         *
        */
        public static BiFunction<Double, Double, Double> TO_DEGREES_FROM_RAW = (raw, offset) -> {
            return ((raw * 360) + (360 - offset)) % 360; //Raw -> Degrees -> positive offset -> wrap degrees
        };
       //public static double ConversionFactor = -360;

        public static class ConversionOffset {
            // y = m*x + b
            // Radians = slope * magnitude + offset
            //0.22047840551196013
            public static double FrontLeft = 79.37222598;
            //0.4913601122840028
            public static double FrontRight = 176.88964042;
            //0.8514951712873793
            public static double BackLeft = 306.53826166;
            //0.10443570261089256
            public static double BackRight = 37.59685294;
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
}
