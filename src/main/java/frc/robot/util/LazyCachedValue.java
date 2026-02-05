package frc.robot.util;

import java.util.function.Supplier;

public class LazyCachedValue<T> extends LazyValue<T> {
    protected T cachedValue;

    public LazyCachedValue(Supplier<T> tFactory) {
        super(tFactory);
    }

    @Override
    public T get() {
        if(cachedValue == null) this.cachedValue = super.get();
        return this.cachedValue;
    }
}
