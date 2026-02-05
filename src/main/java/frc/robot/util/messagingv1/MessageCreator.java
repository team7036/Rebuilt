package frc.robot.util.messagingv1;

/**
 * Create simple messages for being sent
 * Implement and define based on specific message.
 */
public interface MessageCreator {
    ExpandingByteBuffer create();
}
