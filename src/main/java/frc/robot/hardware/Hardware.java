package frc.robot.hardware;

import java.util.HashMap;
import java.util.Map;

public class Hardware {
    private final Map<Token, HardwareHolder<?>> hardwareCreators;

    public Hardware() {
        this.hardwareCreators = new HashMap<>();
    }

    public <T extends Motor> void registerMotorType(Class<?> token, Class<T> type, Factory<T> creator) {
        this.registerType(token, HardwareType.MOTOR, type, creator);
    }

    public <T extends Encoder> void registerEncoderType(Class<?> token, Class<T> type, Factory<T> creator) {
        this.registerType(token, HardwareType.ENCODER, type, creator);
    }

    public <T extends Gyro> void registerGyroType(Class<?> token, Class<T> type, Factory<T> creator) {
        this.registerType(token, HardwareType.GYRO, type, creator);
    }

    private <T extends HardwareDevice> void registerType(Class<?> token, HardwareType hardwareType, Class<T> type, Factory<T> creator) {
        this.hardwareCreators.put(
                new Token(token, hardwareType),
                new HardwareHolder<>(type, creator)
        );
    }

    public Motor createMotor(Class<?> clazz, int id) {
        Token token = new Token(clazz, HardwareType.MOTOR);
        if(!this.hardwareCreators.containsKey(token)) throw new IllegalArgumentException("No registered motor for token %s!".formatted(clazz.getSimpleName()));
        HardwareHolder<?> holder = this.hardwareCreators.get(token);
        if(!(Motor.class.isAssignableFrom(holder.type()))) throw new IllegalStateException("Hardware instance assigned to token %s is not of type Motor!".formatted(clazz.getSimpleName()));
        //noinspection unchecked
        return ((HardwareHolder<? extends Motor>) holder).creator().create(this, id);
    }

    public Encoder createEncoder(Class<?> clazz, int id) {
        Token token = new Token(clazz, HardwareType.ENCODER);
        if(!this.hardwareCreators.containsKey(token)) throw new IllegalArgumentException("No registered encoder for token %s!".formatted(clazz.getSimpleName()));
        HardwareHolder<?> holder = this.hardwareCreators.get(token);
        if(!(Encoder.class.isAssignableFrom(holder.type()))) throw new IllegalStateException("Hardware instance assigned to token %s is not of type Encoder!".formatted(clazz.getSimpleName()));
        //noinspection unchecked
        return ((HardwareHolder<? extends Encoder>) holder).creator().create(this, id);
    }

    public Gyro createGyro(Class<?> clazz, int id) {
        Token token = new Token(clazz, HardwareType.GYRO);
        if(!this.hardwareCreators.containsKey(token)) throw new IllegalArgumentException("No registered gyro for token %s!".formatted(clazz.getSimpleName()));
        HardwareHolder<?> holder = this.hardwareCreators.get(token);
        if(!(Gyro.class.isAssignableFrom(holder.type()))) throw new IllegalStateException("Hardware instance assigned to token %s is not of type Gyro!".formatted(clazz.getSimpleName()));
        //noinspection unchecked
        return ((HardwareHolder<? extends Gyro>) holder).creator().create(this, id);
    }

    record HardwareHolder<T extends HardwareDevice>(Class<T> type, Factory<T> creator) {}

    record Token(Class<?> classToken, HardwareType type) {}

    enum HardwareType {
        MOTOR,
        ENCODER,
        GYRO
    }

    @FunctionalInterface
    public interface Factory<T> {
        T create(Hardware hardware, int id);
    }
}
