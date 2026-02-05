package frc.robot.util;

import java.util.function.Function;
import java.util.function.Supplier;

public class LazyFunction<T, R> {
    protected Function<T, R> tFactory;

    public LazyFunction(Function<T, R> tFactory) {
        this.tFactory = tFactory;
    }

    public R get(T value) {
        return this.tFactory.apply(value);
    }
}
