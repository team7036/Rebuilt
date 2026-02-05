package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.messagingv1.MessageBus;
import frc.robot.util.messagingv1.MessageChannel;
import frc.robot.util.messagingv1.ReadOnlyByteBuffer;

public record DrivetrainMessageHandler(Drivetrain drivetrain) implements MessageChannel.MessageReceiver {
    @Override
    public void receive(MessageBus bus, MessageChannel drivetrainChannel, String messageId, ReadOnlyByteBuffer data, long timestamp) {
        if(!bus.isLatest(drivetrainChannel, messageId, timestamp)) return;
        if(messageId.equals("drivetrain:vision_update/7036")) {
            double poseX = data.getDouble();
            double poseY = data.getDouble();
            double angleRads = data.getDouble();
            double visionTimestamp = data.getDouble();
            drivetrain.addVisionEstimation(
                    new Pose2d(new Translation2d(poseX, poseY), Rotation2d.fromRadians(angleRads)),
                    visionTimestamp
            );
        }
    }
}
