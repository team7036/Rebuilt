package frc.robot.subsystems.drive;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Swerve;

public class Drivetrain extends SubsystemBase {

    private final SwerveModule frontLeft, frontRight, backLeft, backRight;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private SwerveModuleState[] moduleStates;

    private final ADXRS450_Gyro gyro;

    private final double maxSpeed = Constants.Drivetrain.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond);
    private boolean fieldRelative = false;

    public Drivetrain() {

        frontLeft = new SwerveModule(
            "FrontLeftSwerve", 
            IDs.CAN.FrontLeftDrive, 
            IDs.CAN.FrontLeftTurn, 
            IDs.DIO.FrontLeftEncoder,
            Swerve.EncoderOffset.FrontLeft
        );
        frontRight = new SwerveModule(
            "FrontRightSwerve", 
            IDs.CAN.FrontRightDrive, 
            IDs.CAN.FrontRightTurn, 
            IDs.DIO.FrontRightEncoder,
            Swerve.EncoderOffset.FrontRight
        );
        backLeft = new SwerveModule(
            "BackLeftSwerve", 
            IDs.CAN.BackLeftDrive, 
            IDs.CAN.BackLeftTurn, 
            IDs.DIO.BackLeftEncoder,
            Swerve.EncoderOffset.BackLeft
        );
        backRight = new SwerveModule(
            "BackRightSwerve", 
            IDs.CAN.BackRightDrive, 
            IDs.CAN.BackRightTurn, 
            IDs.DIO.BackRightEncoder,
            Swerve.EncoderOffset.BackRight
        );

        gyro = new ADXRS450_Gyro();

        this.kinematics = new SwerveDriveKinematics(
                Constants.Swerve.Position.FrontLeft,
                Constants.Swerve.Position.FrontRight,
                Constants.Swerve.Position.BackLeft,
                Constants.Swerve.Position.BackRight);

        this.odometry = new SwerveDriveOdometry(
                this.kinematics,
                this.getAngle(),
                this.getPositions());

        this.resetPose();

    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose() {
        odometry.resetPose(getPose());
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(moduleStates);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getAngle()));
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }

    private SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
                this.frontLeft.getPosition(),
                this.frontRight.getPosition(),
                this.backLeft.getPosition(),
                this.backRight.getPosition()
        };
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(this.gyro.getAngle() % 360);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("DrivetrainSubsystem");
        SmartDashboard.putData("drive/swerve/backLeft", backLeft);
        SmartDashboard.putData("drive/swerve/backRight", backRight);
        SmartDashboard.putData("drive/swerve/frontRight", frontRight);
        SmartDashboard.putData("drive/swerve/frontLeft", frontLeft);
    }
}
