package frc.robot.util.messagingv1_1;

import frc.robot.util.messagingv1.ExpandingByteBuffer;
import frc.robot.util.messagingv1.Message;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public abstract class MessageTemplate {
    private final Supplier<Message> builder;

    public MessageTemplate() {
        this.builder = new Message.Constructor(messageId()).withParams(parameters()).gather();
    }

    protected final Message newMessage() {
        return this.builder.get();
    }

    public abstract CreatedMessage create();

    protected abstract String messageId();

    protected abstract List<Class<?>> parameters();

    public record CreatedMessage(Message message, Consumer<ExpandingByteBuffer> bufPropagator) {}

}
