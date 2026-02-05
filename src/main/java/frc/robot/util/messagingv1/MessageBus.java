package frc.robot.util.messagingv1;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class MessageBus {
    private final Map<String, MessageChannel> channels;
    private final Map<String, Long> latestMessageTimestamps;
    private final ExecutorService messageService;

    public MessageBus(MessageExecutorType type) {
        this.channels = new ConcurrentHashMap<>();
        this.latestMessageTimestamps = new ConcurrentHashMap<>();
        this.messageService = switch (type) {
            case SINGLE -> Executors.newSingleThreadExecutor();
            case DOUBLE -> Executors.newFixedThreadPool(2);
            case WORK_STEALING -> Executors.newWorkStealingPool(2);
        };
    }

    void registerChannel(MessageChannel channel) {
        this.channels.put(channel.channelId(), channel);
    }

    public MessageChannel getChannel(String channelId) {
        return this.channels.get(channelId);
    }

    void sendMessage(MessageChannel channel, Message message) {
        String tsKey = channel.channelId() + ":" + message.messageId();

        boolean isLatest = latestMessageTimestamps.compute(
                tsKey,
                (k, lastTs) -> {
                    if (lastTs == null || message.timestamp() > lastTs) {
                        return message.timestamp();
                    }
                    return lastTs;
                }
        ) == message.timestamp();

        if (!isLatest) return;

        this.messageService.submit(() -> handleMessage(channel, message));
    }

    private void handleMessage(MessageChannel channel, Message message) {
        channel.receiver()
                .receive(
                        this, channel, message.messageId(),
                        message.readOnlyBuffer(), message.timestamp()
                );
    }

    public boolean isLatest(MessageChannel channel, String messageId, long timestamp) {
        String tsKey = channel.channelId() + ":" + messageId;
        Long latest = this.latestMessageTimestamps.get(tsKey);
        return latest != null && latest == timestamp;
    }

    public void shutdown() {
        this.messageService.shutdown();

        try {
            if(!this.messageService.awaitTermination(60, TimeUnit.SECONDS)) {
                this.messageService.shutdownNow();
            }
        } catch (InterruptedException e) {
            this.messageService.shutdownNow();
            Thread.currentThread().interrupt();
        }
    }

    public enum MessageExecutorType {
        SINGLE,
        DOUBLE,
        WORK_STEALING
    }
}
