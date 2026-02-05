package frc.robot.util.sendablesv1;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;

public abstract class SendingSubsystem extends SubsystemBase {
    private final Map<Property<?>, String> properties = new HashMap<>();

    public SendingSubsystem() {
        super();
    }

    public SendingSubsystem(String name) {
        super(name);
    }

    protected final <T> T addProperty(String fieldName, String dashboardKey, T value) {
        if(properties.containsValue(dashboardKey)) throw new IllegalArgumentException("Dashboard Key already registered in subsystem!");
        Property<T> property = new Property<>(this, fieldName);
        this.properties.put(property, dashboardKey);

        return value;
    }

    @Override
    public final void initSendable(SendableBuilder builder) {
        this.properties.keySet().forEach(p -> p.setup(builder, this.properties.get(p)));
    }
}
