package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

/**
 * The Vision Subsystem for the Robot
 * <br>
 * </br>
 * Interacts with the robot's limelight to provide the drivetrain with a vision
 * pose estimate
 */
public class Vision extends SubsystemBase {
    // If the limelight hasn't disconnected, this will be true
    private static boolean active = false;

    // What tags to detect for, limelight only tracks ids placed here
    private final int[] checkedTags = { 9, 10, 5, 8 };

    /* hadHearbeat - If it had a heartbeat in the previous check
     * prevHeartbeat - What the last heartbeat value was
     * deadChecks - How many checks were dead (no heartbeat) */
    private boolean hadHeartbeat = false;
    private double prevHeartbeat = -1;
    private int deadChecks = 0;

    public Vision() {
        super("Vision");

        // Limits crop window for improved performance
        LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);

        // Change the camera pose relative to robot center (x forward, y left, z up,
        // degrees)
        LimelightHelpers.setCameraPose_RobotSpace("",
                0.5, // Forward offset (meters)
                0.0, // Side offset (meters)
                0.5, // Height offset (meters)
                0.0, // Roll (degrees)
                0.0, // Pitch (degrees)
                0.0 // Yaw (degrees)
        );
        LimelightHelpers.setFiducial3DOffset("",
                0.0, // Forward offset
                0.0, // Side offset
                0.5 // Height offset
        );
        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride("", this.checkedTags); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride("", 2.0f); // Process at half resolution

        // Adjust keystone crop window (-0.95 to 0.95 for both horizontal and vertical)
        LimelightHelpers.setKeystone("", 0.1, -0.05);

        // Marks the limelight as active/connected
        active = true;
    }

    @Override
    public void periodic() {
        // If the limelight disconnected, ignore periodic
        if (!active)
            return;

        // Check Heartbeat for disconnection
        checkHeartbeat();

        // If has valid target
        if (LimelightHelpers.getTV("")) {
            PoseEstimate estimate = Robot.isBlue() ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("")
                    : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("");
        }
    }

    /**
     * Checks if the heartbeat is valid
     * <br>
     * </br>
     * {@code LimelightHelpers.getHeartbeat("")} returns a double, representing the
     * heartbeat of the limelight.
     * The double returned increases every frame (typically 90 fps for older
     * versions).
     * If the double isn't greater than the previous heartbeat, it checks if in the
     * last frame it had a heartbeat.
     * If the limelight didnt't have a heartbeat, it increases {@code deadChecks}.
     * If the {@code deadChecks} field is greater or equal to the
     * {@code heartbeatLeniency} field, the limelight is considered
     * disconnected and {@code active} is set to false, stopping the
     * {@code Vision.periodic()} method from continuing.
     */
    private void checkHeartbeat() {
        double curBeat = LimelightHelpers.getHeartbeat("");
        if (curBeat > prevHeartbeat) {
            deadChecks = 0;
            hadHeartbeat = true;
            prevHeartbeat = curBeat;
        } else {
            if (!hadHeartbeat) {
                deadChecks++;
            }
            if (deadChecks >= Constants.Vision.HEARTBEAT_LENIENCY) {
                active = false;
                return;
            }
            hadHeartbeat = false;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Vision");
        builder.addBooleanProperty("Has Valid Target", () -> LimelightHelpers.getTV(""), null);
    }
}
