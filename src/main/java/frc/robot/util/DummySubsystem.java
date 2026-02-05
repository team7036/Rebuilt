package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DummySubsystem extends SubsystemBase {
    private static int ID = 0;

    public DummySubsystem() {
        this("DUMMY-" + ID);
        ID++;
    }

    public DummySubsystem(String name) {
        super(name);
    }
}
