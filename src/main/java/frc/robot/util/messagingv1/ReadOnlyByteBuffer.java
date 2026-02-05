package frc.robot.util.messagingv1;

import java.nio.ByteBuffer;

@SuppressWarnings("ClassCanBeRecord")
public class ReadOnlyByteBuffer {
    private final ByteBuffer ro;

    public ReadOnlyByteBuffer(ByteBuffer buffer) {
        this.ro = buffer.asReadOnlyBuffer();
        this.ro.flip();
    }

    public int getInt() {
        return this.ro.getInt();
    }
    public double getDouble() {
        return this.ro.getDouble();
    }
    public float getFloat() {
        return this.ro.getFloat();
    }
    public long getLong() {
        return this.ro.getLong();
    }
    public short getShort() {
        return this.ro.getShort();
    }
    public char getChar() {
        return this.ro.getChar();
    }
    public byte getByte() {
        return this.ro.get();
    }
    public boolean getBool() {
        return getByte() != 0;
    }
}
