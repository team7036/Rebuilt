// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public final class Constants {

    public final class IDs {

        public final class Controllers {
            public static int Driver = 0;
            public static int Operator = 1;
        }

        public final class DIO {
            public static int FrontLeftEncoder = 0;
            public static int FrontRightEncoder = 1;
            public static int BackLeftEncoder = 3;
            public static int BackRightEncoder = 4;

            public static int ShooterFuelSensor = 9;
        }

        public final class CAN {
            // Swerve
            public static int FrontLeftDrive = 10;
            public static int FrontLeftTurn = 11;
            public static int FrontRightDrive = 12;
            public static int FrontRightTurn = 13;
            public static int BackLeftDrive = 14;
            public static int BackLeftTurn = 15;
            public static int BackRightDrive = 16;
            public static int BackRightTurn = 17;
            // Shooter
            public static int ShooterLeader = 51;
            public static int ShooterFollower = 55;
            public static int ShooterStaging = 40;
        }
    }

    public final class Shooter {
        public static double STAGING_SPEED = -0.2;
    }

    public final class Drivetrain {
        // Swerve Module Specs
        public static final AngularVelocity FREE_SPEED_RPM = Units.RPM.of(108); // rpm
        public static final Distance WHEEL_DIAMETER = Distance.ofBaseUnits(4, Units.Inches); // 4 inches
        public static final Distance WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.times(Math.PI) ; // meters
        public static final double DRIVE_GEAR_RATIO = 6.12; 
        // TODO Measure this through experimentation
        public static final LinearVelocity MAX_LINEAR_VELOCITY = LinearVelocity.ofBaseUnits(2.0, Units.MetersPerSecond);
        // TODO Measure this through experimentation
        public static AngularVelocity MAX_ANGULAR_VELOCITY = AngularVelocity.ofBaseUnits(2.0, Units.RadiansPerSecond);
        public static final double MOTOR_ROTATIONS_PER_METER = WHEEL_CIRCUMFERENCE.in(Units.Meters) * Math.PI * DRIVE_GEAR_RATIO;
    }

    public final class Swerve {

       public static double EncoderFullRange = 2*Math.PI;

        public static class EncoderOffset {
            public static double FrontLeft = 0.6348880158722005 ;
            public static double FrontRight = 0.4487665112191628;
            public static double BackLeft = 0.2867699571692489;
            public static double BackRight = 0.10833915270847881;
        }

        public static class Position {
            
            public static Translation2d FrontLeft = new Translation2d(0.2794,0.2794);
            public static Translation2d FrontRight  = new Translation2d(0.2794,-0.2794);
            public static Translation2d BackLeft = new Translation2d(-0.2794,0.2794);
            public static Translation2d BackRight = new Translation2d(-0.2794,-0.2794);
        }

        public static class Feedforward {
            public static class Drive {
                public static double kS = 0;
                public static double kV = 0;
            }
            public static class Turn {
                public static double kS = 0;
                public static double kV = 0;
            }
        }
        public static class PID {
            public static class Drive {
                public static double kP = 10;
                public static double kI = 0;
                public static double kD = 0;
            }
            public static class Turn {
                public static double kP = 12;
                public static double kI = 0;
                public static double kD = 0;
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
        public static int INTAKE_ANGLE_MOTOR_ID = 60;
        public static int INTAKE_FLYWHEEL_MOTOR_ID = 45;

        public static double ANGLE_MAX_VELOCITY = 5.0;
        public static double ANGLE_MAX_ACCELERATION = 9.0;

        public static double POSITION_CONVERSION_FACTOR = 0.05;
        public static double VELOCITY_CONVERSION_FACTOR = 0.00084;

        public static double STOWED_ANGLE = 0.1;
        public static double OFFSET_ANGLE = 2.425; // The measure angle where the arm is parallel with the floor
        public static double LOWERED_THRESHOLD = 2.6; // The angle where the intake flywheel can start
        public static double SHOOT_THRESHOLD = 1.0; // The angle where the intake flywheel can start
        public static double INTAKING_ANGLE = 2.7;
        public static double FLYWHEEL_SPEED = -1;
        

        public static class AnglePID {
            public static double kP = 15.0;
            public static double kI = 0;
            public static double kD = 0;
        }

        public static class AngleFeedForward {
            public static double kS = 0.0; // volts, static gain that keeps the arm from moving
            public static double kG = 0.3596; // volts, how much voltage it needs to overcome gravity
            public static double kV = 0.4; // volts * seconds / radians, how quickly the arm moves for every volt
        }
    }

    public static class Controllers {
        public static final int DRIVER_PORT = 0;
    }
}
