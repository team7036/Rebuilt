package frc.robot.util.messagingv1;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Message {
    private final String messageId;
    private final ExpandingByteBuffer data;
    private final long timestamp;
    private ReadOnlyByteBuffer ro;

    private boolean used = false;

    private Message(String messageId, MessageStruct struct) {
        this.messageId = messageId;
        this.data = ExpandingByteBuffer.ofTypes(struct.params());
        this.timestamp = System.currentTimeMillis();
    }

    public Message(String messageId, MessageCreator creator) {
        this.messageId = messageId;
        this.data = creator.create();
        this.timestamp = System.currentTimeMillis();
    }

    public void sendMessage(MessageChannel channel, Consumer<ExpandingByteBuffer> bufPropagator) {
        if (used) throw new IllegalStateException("Cannot re-send message after use!");
        bufPropagator.accept(this.data);
        compileReadOnlyBuffer();
        channel.sendMessage(this);
        this.used = true;
    }

    private void compileReadOnlyBuffer() {
        this.ro = this.data.read();
    }

    public String messageId() {
        return messageId;
    }

    public ReadOnlyByteBuffer readOnlyBuffer() {
        return ro;
    }

    public long timestamp() {
        return this.timestamp;
    }

    public record MessageStruct(List<Class<?>> params) {}

    public static class Builder {
        private final String messageId;
        private final List<Class<?>> types;

        public Builder(String messageId) {
            this.messageId = messageId;
            this.types = new ArrayList<>();
        }

        public Builder addParam(Class<?> type) {
            this.types.add(type);
            return this;
        }

        public Builder withParams(List<Class<?>> types) {
            this.types.addAll(types);
            return this;
        }

        public Message build() {
            return new Message(
                    this.messageId,
                    new MessageStruct(this.types)
            );
        }
    }
    public static class Constructor {
        private final String messageId;
        private final List<Class<?>> types;

        public Constructor(String messageId) {
            this.messageId = messageId;
            this.types = new ArrayList<>();
        }

        public Constructor addParam(Class<?> type) {
            this.types.add(type);
            return this;
        }

        public Constructor withParams(List<Class<?>> types) {
            this.types.addAll(types);
            return this;
        }

        public Supplier<Message> gather() {
            List<Class<?>> frozen = List.copyOf(this.types);
            return () -> new Message(
                    this.messageId,
                    new MessageStruct(frozen)
            );
        }
    }

}
