// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {


    public final class Swerve {

        public static final SwerveConfig FrontLeft = new SwerveConfig(0, 0, 0, 0);
        public static final SwerveConfig FrontRight = new SwerveConfig(0, 0, 0, 0);
        public static final SwerveConfig BackLeft = new SwerveConfig(0, 0, 0, 0);
        public static final SwerveConfig BackRight = new SwerveConfig(0, 0, 0, 0);

        public static class SwerveConfig {
            public final int turnMotorId;
            public final int turnEncoderId;
            public final int driveMotorId;
            public final int driveEncoderId;
            public SwerveConfig(int turnMotorId, int turnEncoderId, int driveMotorId, int driveEncoderId){
                this.turnMotorId = turnMotorId;
                this.turnEncoderId = turnEncoderId;
                this.driveMotorId = driveMotorId;
                this.driveEncoderId = driveEncoderId;
            }
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

    public static class Controllers {
        public static final int DRIVER_PORT = 0;
    }
}
