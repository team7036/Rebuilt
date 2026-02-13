package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

    private final SwerveModule frontLeft, frontRight, backLeft, backRight;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private SwerveModuleState[] moduleStates;

    private final ADXRS450_Gyro gyro;

    private final double maxSpeed = Constants.Drivetrain.maxSpeed;
    private boolean fieldRelative = false;

    public Drivetrain() {

        frontLeft = new SwerveModule(Constants.Swerve.FrontLeft);
        frontRight = new SwerveModule(Constants.Swerve.FrontRight);
        backLeft = new SwerveModule(Constants.Swerve.BackLeft);
        backRight = new SwerveModule(Constants.Swerve.BackRight);

        gyro = new ADXRS450_Gyro();

        this.kinematics = new SwerveDriveKinematics(
            Constants.Swerve.Position.FrontLeft,
            Constants.Swerve.Position.FrontRight,
            Constants.Swerve.Position.BackLeft,
            Constants.Swerve.Position.BackRight
        );

        this.odometry = new SwerveDriveOdometry(
                this.kinematics,
                this.getAngle(),
                this.getPositions()
        );

        this.resetPose();

    }

    // PathPlannerMethods
    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
    public void resetPose() {
        odometry.resetPose(getPose());
    }
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return kinematics.toChassisSpeeds(moduleStates);
    }
    public void driveRobotRelative(ChassisSpeeds speeds){
        moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getAngle()));
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }

    private SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[]{
                this.frontLeft.getPosition(),
                this.frontRight.getPosition(),
                this.backLeft.getPosition(),
                this.backRight.getPosition()
        };
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRadians(this.gyro.getAngle());
    }
}

