package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.hardware.Gyro;
import frc.robot.hardware.Hardware;
import frc.robot.hardware.sysid.SysIdRoutine;
import frc.robot.hardware.sysid.SysIdRoutineContainer;
import frc.robot.util.messagingv1.MessageChannel;
import frc.robot.util.messagingv1_1.MessageChannelOwner;

import java.util.Arrays;
import java.util.function.Supplier;

public class Drivetrain extends SubsystemBase implements MessageChannelOwner {

    private final SwerveModule frontLeft, frontRight, backLeft, backRight;

    private final Gyro gyro;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final Estimation estimation;
    private final Speed speed;

    private boolean fieldRelative = false;

    private final MessageChannel channel;

    private final SysIdRoutineContainer sysIdContainer;
    private final SysIdRoutine sysIdRoutine;


    public Drivetrain(RobotContainer container) {
        Hardware hardware = container.getHardware();

        this.frontLeft = new SwerveModule(hardware, Constants.Swerve.Hardware.FRONT_LEFT);
        this.frontRight = new SwerveModule(hardware, Constants.Swerve.Hardware.FRONT_RIGHT);
        this.backLeft = new SwerveModule(hardware, Constants.Swerve.Hardware.BACK_LEFT);
        this.backRight = new SwerveModule(hardware, Constants.Swerve.Hardware.BACK_RIGHT);

        this.gyro = hardware.createGyro(ADXRS450_Gyro.class, 0); //Replace with ACTUAL gyro creation

        this.kinematics = new SwerveDriveKinematics(
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

        PathFollowingController pathController = switch(Constants.Swerve.CONTROLLER_TYPE) {
            case PID -> new PPHolonomicDriveController(
                    Constants.Swerve.Pid.DRIVE.toPathPlannerConstants(),
                    Constants.Swerve.Pid.TURN.toPathPlannerConstants()
            );
            case LQR -> new PPLTVController(
                    Constants.Swerve.LQR.DT,
                    this.speed.maxSpeed()
            );
        };


        if(Constants.Swerve.USE_AUTO)
            AutoBuilder.configure(
                    this.estimation::estimatePose,
                    this.estimation::resetPose,
                    this.speed::robotRelative,
                    this.speed::driveRobotRelative,
                    pathController,
                    Constants.Swerve.PATHPLANNER_CONFIG,
                    () -> true,
                    this
            );

        this.channel = container.createChannel(
                "subsystem/drivetrain",
                new DrivetrainMessageHandler(this)
        );

        //Setup SysIdRoutineContainer with motors

        this.sysIdContainer = new SysIdRoutineContainer();
        this.frontLeft.populateContainer("FL", this.sysIdContainer);
        this.frontRight.populateContainer("FR", this.sysIdContainer);
        this.backLeft.populateContainer("BL", this.sysIdContainer);
        this.backRight.populateContainer("BR", this.sysIdContainer);

        //Setup and Complete SysIdRoutine with test runs

        this.sysIdRoutine = new SysIdRoutine();
        this.sysIdRoutine.addRoutine(
                SysIdRoutineContainer.BuiltSubRoutine.build(
                        this.sysIdContainer,
                        SysIdRoutineContainer.RoutineType.QUASISTATIC,
                        SysIdRoutineContainer.Direction.FORWARD
                )
        ).addRoutine(
                SysIdRoutineContainer.BuiltSubRoutine.build(
                        this.sysIdContainer,
                        SysIdRoutineContainer.RoutineType.QUASISTATIC,
                        SysIdRoutineContainer.Direction.BACKWARD
                )
        ).addRoutine(
                SysIdRoutineContainer.BuiltSubRoutine.build(
                        this.sysIdContainer,
                        SysIdRoutineContainer.RoutineType.DYNAMIC,
                        SysIdRoutineContainer.Direction.FORWARD
                )
        ).addRoutine(
                SysIdRoutineContainer.BuiltSubRoutine.build(
                        this.sysIdContainer,
                        SysIdRoutineContainer.RoutineType.DYNAMIC,
                        SysIdRoutineContainer.Direction.BACKWARD
                )
        );
    }

    private SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[]{
                this.frontLeft.getModuleState().asPosition(),
                this.frontRight.getModuleState().asPosition(),
                this.backLeft.getModuleState().asPosition(),
                this.backRight.getModuleState().asPosition()
        };
    }

    public double[] getTurnEncoderPositions() {
        return new double[] {
            this.frontLeft.getTurnPosition(),
            this.frontRight.getTurnPosition(),
            this.backLeft.getTurnPosition(),
            this.backRight.getTurnPosition()
        };
    }

    public void drive(ChassisSpeeds speeds) {
        this.speed.drive(speeds);
    }

    public double maxSpeed() {
        return this.speed.maxSpeed();
    }

    public SysIdRoutine sysIdRoutine() {
        return this.sysIdRoutine;
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

    @Override
    public MessageChannel getChannel() {
        return this.channel;
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

