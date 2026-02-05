package frc.robot.util.messagingv1;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class ExpandingByteBuffer {
    private static final int MAX_CAPACITY = 2048;
    private static final Map<Class<?>, Integer> BYTES_CACHE = new ConcurrentHashMap<>();

    private ByteBuffer buf;

    public ExpandingByteBuffer(int startingSize) {
        if (startingSize > MAX_CAPACITY) this.buf = ByteBuffer.allocate(MAX_CAPACITY);
        else this.buf = ByteBuffer.allocate(startingSize);
        this.buf.order(ByteOrder.LITTLE_ENDIAN);
    }

    public ExpandingByteBuffer putInt(int value) {
        createSpace(int.class);
        this.buf.putInt(value);
        return this;
    }

    public ExpandingByteBuffer putDouble(double value) {
        createSpace(double.class);
        this.buf.putDouble(value);
        return this;
    }

    public ExpandingByteBuffer putFloat(float value) {
        createSpace(float.class);
        this.buf.putFloat(value);
        return this;
    }

    public ExpandingByteBuffer putLong(long value) {
        createSpace(long.class);
        this.buf.putLong(value);
        return this;
    }

    public ExpandingByteBuffer putShort(short value) {
        createSpace(short.class);
        this.buf.putShort(value);
        return this;
    }

    public ExpandingByteBuffer putChar(char value) {
        createSpace(char.class);
        this.buf.putChar(value);
        return this;
    }

    public ExpandingByteBuffer putByte(byte value) {
        createSpace(1);
        this.buf.put(value);
        return this;
    }

    public ExpandingByteBuffer putBool(boolean value) {
        createSpace(1);
        this.buf.put((byte) (value ? 1 : 0));
        return this;
    }

    public ReadOnlyByteBuffer read() {
        return new ReadOnlyByteBuffer(this.buf);
    }

    private void createSpace(Class<?> type) {
        int typeSize = computeSize(type);
        this.createSpace(typeSize);
    }
    private void createSpace(int typeSize) {
        int remainingCapacity = this.buf.remaining();
        if(remainingCapacity < typeSize) {
            int needed = typeSize - remainingCapacity;
            int grow = Math.max(needed, this.buf.capacity());
            this.expand(grow);
        }
    }

    private static int computeSize(Class<?> type) {
        //Precompute values
        if (type == int.class) return Integer.BYTES;
        if (type == double.class) return Double.BYTES;
        if (type == float.class) return Float.BYTES;
        if (type == long.class) return Long.BYTES;
        if (type == short.class) return Short.BYTES;
        if (type == char.class) return Character.BYTES;
        if (type == byte.class) return 1;
        if (type == boolean.class) return 1;

        Integer cached = BYTES_CACHE.get(type);
        if (cached != null) return cached;

        try {
            int bytes = getBytes(type);
            BYTES_CACHE.put(type, bytes);
            return bytes;
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(
                    type.getSimpleName() + " must define a BYTES field",
                    e
            );
        } catch (IllegalAccessException e) {
            throw new RuntimeException(
                    "Cannot access BYTES field in " + type.getSimpleName(),
                    e
            );
        }
    }

    private static int getBytes(Class<?> type) throws NoSuchFieldException, IllegalAccessException {
        Field f = type.getDeclaredField("BYTES");

        int mods = f.getModifiers();
        if (!Modifier.isStatic(mods) || !Modifier.isFinal(mods) || f.getType() != int.class) {
            throw new RuntimeException(
                    type.getSimpleName() + ".BYTES must be 'static final int'"
            );
        }

        f.setAccessible(true);
        int bytes = f.getInt(null);

        if (bytes <= 0) {
            throw new RuntimeException(
                    type.getSimpleName() + ".BYTES must be > 0"
            );
        }
        return bytes;
    }


    private void expand(int additional) {
        int required = this.buf.capacity() + additional;

        if (required > MAX_CAPACITY) throw new IllegalArgumentException("Cannot expand beyond " + MAX_CAPACITY + " bytes!");

        ByteBuffer expanded = ByteBuffer.allocate(required)
                .order(ByteOrder.LITTLE_ENDIAN);
        this.buf.flip();
        expanded.put(this.buf);
        this.buf = expanded;
    }

    public static ExpandingByteBuffer ofTypes(List<Class<?>> inputTypes) {
        int neededSize = 0;
        for (Class<?> type : inputTypes) {
            neededSize += computeSize(type);
        }
        if(neededSize > MAX_CAPACITY) throw new IllegalArgumentException("Cannot create buffer beyond " + MAX_CAPACITY + " bytes!");
        return new ExpandingByteBuffer(neededSize);
    }

}
