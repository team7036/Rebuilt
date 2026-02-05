package frc.robot.util;

import java.util.function.Function;
import java.util.function.Supplier;

public class LazyCachedFunction<T, R> extends LazyFunction<T, R> {
    protected R cached;

    public LazyCachedFunction(Function<T, R> tFactory) {
        super(tFactory);
    }

    @Override
    public R get(T value) {
        if (this.cached == null) this.cached = super.get(value);
        return this.cached;
    }
}
