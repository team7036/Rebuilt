package frc.robot.util;

import java.util.function.Supplier;

public class LazyValue<T> {
    protected Supplier<T> tFactory;

    public LazyValue(Supplier<T> tFactory) {
        this.tFactory = tFactory;
    }

    public T get() {
        return this.tFactory.get();
    }
}
