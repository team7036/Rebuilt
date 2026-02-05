package frc.robot.util.messagingv1;

public record MessageChannel(MessageBus manager, String channelId, MessageReceiver receiver) {
    public MessageChannel(MessageBus manager, String channelId, MessageReceiver receiver) {
        this.channelId = channelId;
        this.receiver = receiver;
        this.manager = manager;
        this.manager.registerChannel(this);
    }

    void sendMessage(Message message) {
        manager.sendMessage(this, message);
    }

    @FunctionalInterface
    public interface MessageReceiver {
        void receive(MessageBus sendingManager, MessageChannel receivingChannel, String messageId, ReadOnlyByteBuffer buf, long timestamp);
    }
}
