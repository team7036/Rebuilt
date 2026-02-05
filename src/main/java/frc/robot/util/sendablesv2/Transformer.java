package frc.robot.util.sendablesv2;

@FunctionalInterface
public interface Transformer<T> {

    T constrain(T old);

    class BlankTransformer implements Transformer<Object> {
        public static final BlankTransformer INSTANCE = new BlankTransformer();
        private BlankTransformer() {}

        @Override
        public Object constrain(Object old) {
            return old;
        }
    }
}
