// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

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

    public final class Shooter {
        // max rpm of a neo
        public static AngularVelocity maxSpeed = AngularVelocity.ofBaseUnits(5676, Units.RPM);
        public static double thresholdRatio = 0.4;
        public static AngularVelocity thresholdSpeed = maxSpeed.times(thresholdRatio);
        public static int shooterCANId = 60; 
        public static int stagingCANId = 1; //TODO
        public static double ks = 6.0;
        public static double kv;
        public static double maxAmps = 100;
        public static double ampRatio = 0.55;
        public static double mcl = maxAmps * ampRatio;
        public static int motorCurrentLimits = (int) mcl;
    }

    public static class Controllers {
        public static final int DRIVER_PORT = 0;
    }
}
