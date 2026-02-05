package frc.robot.util.messagingv1_1;

import frc.robot.util.messagingv1.Message;
import frc.robot.util.messagingv1.MessageChannel;

public record MessageTarget(MessageChannel channel) {

    public void send(MessageTemplate.CreatedMessage createdMessage) {
        Message message = createdMessage.message();
        message.sendMessage(channel, createdMessage.bufPropagator());
    }

    public static MessageTarget targetOwner(MessageChannelOwner owner) {
        return new MessageTarget(owner.getChannel());
    }
}
