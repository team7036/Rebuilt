package frc.robot.util.sendablesv2;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.reflect.Field;
import java.util.*;

public abstract class SendingSubsystem extends SubsystemBase {
    private final List<Property<?>> properties;

    public SendingSubsystem() {
        this(null);
    }

    public SendingSubsystem(String name) {
        this.properties = new ArrayList<>();
        String actual = name == null ? this.getClass().getSimpleName() : name;
        SendableRegistry.addLW(this, actual, actual);
        CommandScheduler.getInstance().registerSubsystem(this);
        compileProperties();
    }

    private void compileProperties() {
        Map<Field, DashboardProperty> fieldProperties = new HashMap<>();
        getDashboardPropertyFields().forEach(f -> fieldProperties.put(f, f.getAnnotation(DashboardProperty.class)));
        fieldProperties.forEach((field, property) -> {
            properties.add(new Property<>(this, field, property.value(), createConstrainer(property.transformer())));
        });
    }

    private List<Field> getDashboardPropertyFields() {
        return Arrays.stream(this.getClass().getDeclaredFields())
                .filter(f -> f.isAnnotationPresent(DashboardProperty.class))
                .toList();
    }

    private Transformer<?> createConstrainer(Class<? extends Transformer<?>> clazz) {
        if(clazz.equals(Transformer.BlankTransformer.class)) {
            return Transformer.BlankTransformer.INSTANCE;
        } else {
            try {
                return clazz.getConstructor().newInstance();
            } catch (Exception e) {
                throw new RuntimeException("Constrainer implementation \"%s\" must have a no args constructor!".formatted(clazz.getSimpleName()), e);
            }
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        this.properties.forEach(p -> p.setup(builder));
    }
}
