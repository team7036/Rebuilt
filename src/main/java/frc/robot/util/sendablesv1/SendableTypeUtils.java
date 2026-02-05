package frc.robot.util.sendablesv1;

import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Supplier;


public class SendableTypeUtils {

    public static <T> PropertyAdder<T> getPropertyAdder(String propertyName, Class<?> type, boolean isFinal,
                                                        Supplier<T> getter, Consumer<T> setter) {
        Class<?> varType = normalize(type);
        if(varType.equals(Boolean.class)) {
            return (builder) -> {
                //noinspection unchecked
                builder.addBooleanProperty(
                        propertyName,
                        () -> (Boolean) getter.get(),
                        isFinal ? null : (bool) -> ((Consumer<Boolean>) setter).accept(bool)
                );
            };
        } else if (varType.equals(Integer.class) || varType.equals(Long.class)) {
            boolean isInt = varType.equals(Integer.class);
            return (builder) -> builder.addIntegerProperty(
                    propertyName,
                    () -> ((Number) getter.get()).longValue(),
                    isFinal ? null : (num) -> {
                        if(isInt) {
                            int val = Math.toIntExact(Math.max(Integer.MIN_VALUE, Math.min(num, Integer.MAX_VALUE)));
                            //noinspection unchecked
                            ((Consumer<Integer>) setter).accept(val);
                        } else //noinspection unchecked
                            ((Consumer<Long>) setter).accept(num);
                    }
            );
        } else if (varType.equals(Double.class)) {
            return (builder) -> {
                //noinspection unchecked
                builder.addDoubleProperty(
                        propertyName,
                        () -> (Double) getter.get(),
                        isFinal ? null : (num) -> ((Consumer<Double>) setter).accept(num)
                );
            };
        } else if (varType.equals(Float.class)) {
            return (builder) -> {
                //noinspection unchecked
                builder.addFloatProperty(
                        propertyName,
                        () -> (Float) getter.get(),
                        isFinal ? null : (num) -> ((Consumer<Float>) setter).accept(num)
                );
            };
        } else if (varType.equals(String.class)) {
            return (builder) -> {
                //noinspection unchecked
                builder.addStringProperty(
                        propertyName,
                        (Supplier<String>) getter,
                        isFinal ? null : (Consumer<String>) setter
                );
            };
        } else if (varType.equals(boolean[].class)) {
            return (builder) -> {
                //noinspection unchecked
                builder.addBooleanArrayProperty(
                        propertyName,
                        () -> (boolean[]) getter.get(),
                        isFinal ? null : (arr) -> ((Consumer<boolean[]>) setter).accept(arr)
                );
            };
        } else if (varType.equals(int[].class)) {
            return builder -> {
                //noinspection unchecked
                builder.addIntegerArrayProperty(
                        propertyName,
                        () -> fromIntArr((int[]) getter.get()),
                        isFinal ? null : (arr) -> ((Consumer<int[]>) setter).accept(toIntArr(arr))
                );
            };
        }
        else if (varType.equals(double[].class)) {
            return (builder) -> {
                //noinspection unchecked
                builder.addDoubleArrayProperty(
                        propertyName,
                        () -> (double[]) getter.get(),
                        isFinal ? null : (arr) -> ((Consumer<double[]>) setter).accept(arr)
                );
            };
        } else if (varType.equals(float[].class)) {
            return (builder) -> {
                //noinspection unchecked
                builder.addFloatArrayProperty(
                        propertyName,
                        () -> (float[]) getter.get(),
                        isFinal ? null : (arr) -> ((Consumer<float[]>) setter).accept(arr)
                );
            };
        } else if (varType.equals(String[].class)) {
            return (builder) -> {
                //noinspection unchecked
                builder.addStringArrayProperty(
                        propertyName,
                        () -> (String[]) getter.get(),
                        isFinal ? null : (arr) -> ((Consumer<String[]>) setter).accept(arr)
                );
            };
        }
        throw new IllegalArgumentException("No sendable type for property %s".formatted(propertyName));
    }

    private static Class<?> normalize(Class<?> c) {
        if (!c.isPrimitive()) return c;
        if (c == boolean.class) return Boolean.class;
        if (c == int.class) return Integer.class;
        if (c == double.class) return Double.class;
        if (c == float.class) return Float.class;
        if (c == long.class) return Long.class;
        return c;
    }

    private static long[] fromIntArr(int[] arr) {
        long[] out = new long[arr.length];
        Arrays.setAll(out, i -> (long) arr[i]);
        return out;
    }
    private static int[] toIntArr(long[] arr) {
        int[] out = new int[arr.length];
        Arrays.setAll(out, i -> (int) arr[i]);
        return out;
    }


    @FunctionalInterface
    public interface PropertyAdder<T> {
        void addProperty(SendableBuilder builder);
    }
}
