package frc.robot.util.sendablesv2;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.util.sendablesv1.SendableTypeUtils;

import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.EnumSet;

public class Property<T> {
    private final PropertyHandle<T> handle;
    private final String propertyName;

    public Property(SendingSubsystem subsystem, Field field, String propertyName, Transformer<T> transformer) {
        this.handle = new PropertyHandle<>(subsystem, subsystem.getClass(), field.getName(), transformer);
        this.propertyName = propertyName;
    }

    public T get() {
        return this.handle.get();
    }

    public void set(T val) {
        this.handle.set(val);
    }

    public void setup(SendableBuilder builder) {
        SendableTypeUtils.getPropertyAdder(propertyName, this.handle.varType(), this.handle.getModifiers().contains(FieldModifiers.FINAL),
                        this::get, this::set)
                .addProperty(builder);
    }

    static class PropertyHandle<T> {
        private final VarHandle handle;
        private final Object instance;
        private final EnumSet<FieldModifiers> modifiers;
        private final Class<?> variableType;
        private final Transformer<T> transformer;

        public PropertyHandle(Object inst, Class<?> declarer, String varName, Transformer<T> transformer) {
            this.instance = inst;

            Field var;
            try {
                var = declarer.getDeclaredField(varName);
            } catch (NoSuchFieldException e) {
                throw new RuntimeException("Cannot add property if field does not exist!", e);
            }
            this.modifiers = EnumSet.noneOf(FieldModifiers.class);

            int fieldMods = var.getModifiers();

            if(Modifier.isStatic(fieldMods)) modifiers.add(FieldModifiers.STATIC);
            if(Modifier.isVolatile(fieldMods)) modifiers.add(FieldModifiers.VOLATILE);
            if(Modifier.isFinal(fieldMods)) modifiers.add(FieldModifiers.FINAL);

            this.variableType = var.getType();

            Class<?> varType = var.getType();

            try {
                if (modifiers.contains(FieldModifiers.STATIC)) {
                    this.handle = MethodHandles.privateLookupIn(declarer, MethodHandles.lookup())
                            .findStaticVarHandle(declarer, varName, varType);
                } else {
                    this.handle = MethodHandles.privateLookupIn(declarer, MethodHandles.lookup())
                            .findVarHandle(declarer, varName, varType);
                }
            } catch (Exception e) {
                throw new RuntimeException("Failed to get VarHandle for field %s!".formatted(var), e);
            }
            this.transformer = transformer;
        }

        public T get() {
            if (modifiers.contains(FieldModifiers.VOLATILE)) {
                if(modifiers.contains(FieldModifiers.STATIC)) //noinspection unchecked
                    return (T) this.handle.getVolatile();
                else //noinspection unchecked
                    return (T) this.handle.getVolatile(this.instance);
            } else {
                if(modifiers.contains(FieldModifiers.STATIC)) //noinspection unchecked
                    return (T) this.handle.get();
                else //noinspection unchecked
                    return (T) this.handle.get(this.instance);
            }
        }

        public void set(T in) {
            T val = transformer.constrain(in);
            if(modifiers.contains(FieldModifiers.FINAL)) throw new IllegalStateException("Cannot set a final variable!");
            if (modifiers.contains(FieldModifiers.VOLATILE)) {
                if(modifiers.contains(FieldModifiers.STATIC)) this.handle.setVolatile(val);
                else this.handle.setVolatile(this.instance, val);
            } else {
                if(modifiers.contains(FieldModifiers.STATIC)) this.handle.set(val);
                else this.handle.set(this.instance, val);
            }
        }

        public Class<?> varType() {
            return this.variableType;
        }

        public EnumSet<FieldModifiers> getModifiers() {
            return this.modifiers;
        }
    }

    public enum FieldModifiers {
        STATIC, FINAL, VOLATILE
    }
}
