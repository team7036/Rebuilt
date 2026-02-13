package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConfig;
import frc.robot.RobotContainer;
import java.util.Arrays;
import java.util.function.Supplier;

public class Drivetrain extends SubsystemBase {

    private final SwerveModule frontLeft, frontRight, backLeft, backRight;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final Gyro gyro;

    

    private final Estimation estimation;
    private final Speed speed;

    private boolean fieldRelative = false;

    public Drivetrain() {

        frontLeft = new SwerveModule(
            Constants.Swerve.CAN.FrontLeft.DRIVE,
            Constants.Swerve.CAN.FrontLeft.TURN
        );
        frontRight = new SwerveModule(
            Constants.Swerve.CAN.FrontRight.DRIVE,
            Constants.Swerve.CAN.FrontRight.TURN
        );
        backLeft = new SwerveModule(
            Constants.Swerve.CAN.BackLeft.DRIVE,
            Constants.Swerve.CAN.BackLeft.TURN
        );
        backRight = new SwerveModule(
            Constants.Swerve.CAN.BackRight.DRIVE,
            Constants.Swerve.CAN.BackRight.TURN
        );


        this.kinematics = new SwerveDriveKinematics(
                new Transform2d(0, 1),
                Constants.Swerve.Hardware.FL_POS,
                Constants.Swerve.Hardware.FR_POS,
                Constants.Swerve.Hardware.BL_POS,
                Constants.Swerve.Hardware.BR_POS
        );

        this.odometry = new SwerveDriveOdometry(
                this.kinematics,
                this.getAngle(),
                this.getPositions()
        );

        this.estimation = new Estimation(
                new SwerveDrivePoseEstimator(
                        this.kinematics,
                        this.getAngle(),
                        this.getPositions(),
                        odometry.getPoseMeters()
                ),
                this.odometry
        );

        this.speed = new Speed(
                this.kinematics,
                this::getAngle,
                this::isFieldRelative,
                frontLeft,
                frontRight,
                backLeft,
                backRight
        );

        this.resetGyro();

    }

    private SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[]{
                this.frontLeft.getModuleState().asPosition(),
                this.frontRight.getModuleState().asPosition(),
                this.backLeft.getModuleState().asPosition(),
                this.backRight.getModuleState().asPosition()
        };
    }

    public void drive(ChassisSpeeds speeds) {
        this.speed.drive(speeds);
    }

    public void testModules() {
        this.frontLeft.test();
        this.frontRight.test();
        this.backLeft.test();
        this.backRight.test();
    }

    public double[] getEncoderValues() {
        return new double[] {
                this.frontLeft.getTurnPosition(),
                this.frontRight.getTurnPosition(),
                this.backLeft.getTurnPosition(),
                this.backRight.getTurnPosition()
        };
    }

    public double maxSpeed() {
        return this.speed.maxSpeed();
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRadians(this.gyro.getAngle());
    }

    private void resetGyro() {
        this.gyro.reset();
    }

    private boolean isFieldRelative() {
        return this.fieldRelative;
    }

    void addVisionEstimation(Pose2d visionPose, double timestamp) {
        this.estimation.addVisionEstimation(visionPose, timestamp);
    }

    record Estimation(SwerveDrivePoseEstimator estimator, SwerveDriveOdometry odometry) {
        public Pose2d estimatePose() {
            return this.estimator.getEstimatedPosition();
        }

        public void resetPose(Pose2d newPose) {
            this.odometry.resetPose(newPose);
        }

        public void addVisionEstimation(Pose2d visionPose, double timestamp) {
            this.estimator.addVisionMeasurement(visionPose, timestamp);
        }
    }

    record Speed(SwerveDriveKinematics kinematics, Supplier<Rotation2d> angle, Supplier<Boolean> isFieldRelative, SwerveModule... modules) {
        public ChassisSpeeds robotRelative() {
            return kinematics.toChassisSpeeds(
                    moduleStates()
            );
        }

        public void drive(ChassisSpeeds speeds) {
            if(isFieldRelative().get())
                this.driveRobotRelative(ChassisSpeeds.fromRobotRelativeSpeeds(
                    speeds, angle.get()
                ));
            else
                this.driveRobotRelative(speeds);
        }

        public void driveRobotRelative(ChassisSpeeds speeds) {
            SwerveModuleState[] requestedStates = kinematics.toSwerveModuleStates(speeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(requestedStates, maxSpeed());
            for (int i = 0; i < modules.length; i++) {
                modules[i].requestState(requestedStates[i]);
            }
        }

        private double maxSpeed() {
            //noinspection OptionalGetWithoutIsPresent
            return Arrays.stream(modules).map(SwerveModule::getMaxSpeed)
                    .reduce(Double::sum).get() / modules.length;
        }

        private SwerveModuleState[] moduleStates() {
            return Arrays.stream(modules).map(module -> module.getModuleState().asState())
                    .toArray(SwerveModuleState[]::new);
        }
    }
}

