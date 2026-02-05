package frc.robot.util.sendablesv1;

import edu.wpi.first.util.sendable.SendableBuilder;

import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

public class Property<T> {
    private final PropertyHandle<T> handle;

    public Property(SendingSubsystem subsystem, String varName) {
        Class<? extends SendingSubsystem> clazz = subsystem.getClass();

        this.handle = new PropertyHandle<>(subsystem, clazz, varName);
    }

    public T get() {
        return this.handle.get();
    }

    public void set(T val) {
        this.handle.set(val);
    }

    public void setup(SendableBuilder builder, String propertyName) {
        SendableTypeUtils.getPropertyAdder(propertyName, this.handle.varType(), this.handle.isVarFinal(), this::get, this::set)
                .addProperty(builder);
    }

    static class PropertyHandle<T> {
        private final VarHandle handle;
        private final Object instance;
        private final boolean isStatic;
        private final boolean isVolatile;
        private final boolean isFinal;
        private final Class<?> variableType;

        public PropertyHandle(Object inst, Class<?> declarer, String varName) {
            this.instance = inst;

            Field var;
            try {
                var = declarer.getDeclaredField(varName);
            } catch (NoSuchFieldException e) {
                throw new RuntimeException("Cannot add property if field does not exist!", e);
            }

            int modifiers = var.getModifiers();

            this.isStatic = Modifier.isStatic(modifiers);
            this.isVolatile = Modifier.isVolatile(modifiers);
            this.isFinal = Modifier.isFinal(modifiers);

            this.variableType = var.getType();

            Class<?> varType = var.getType();

            try {
                if (isStatic) {
                    this.handle = MethodHandles.privateLookupIn(declarer, MethodHandles.lookup())
                            .findStaticVarHandle(declarer, varName, varType);
                } else {
                    this.handle = MethodHandles.privateLookupIn(declarer, MethodHandles.lookup())
                            .findVarHandle(declarer, varName, varType);
                }
            } catch (Exception e) {
                throw new RuntimeException("Failed to get VarHandle for field %s!".formatted(var), e);
            }
        }

        public T get() {
            if (isVolatile) {
                if(isStatic) //noinspection unchecked
                    return (T) this.handle.getVolatile();
                else //noinspection unchecked
                    return (T) this.handle.getVolatile(this.instance);
            } else {
                if(isStatic) //noinspection unchecked
                    return (T) this.handle.get();
                else //noinspection unchecked
                    return (T) this.handle.get(this.instance);
            }
        }

        public void set(T val) {
            if(isFinal) throw new IllegalStateException("Cannot set a final variable!");
            if (isVolatile) {
                if(isStatic) this.handle.setVolatile(val);
                else this.handle.setVolatile(this.instance, val);
            } else {
                if(isStatic) this.handle.set(val);
                else this.handle.set(this.instance, val);
            }
        }

        public boolean isVarFinal() {
            return this.isFinal;
        }

        public Class<?> varType() {
            return this.variableType;
        }
    }
}
