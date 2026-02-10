package frc.robot.util.periodicsv1;

import java.lang.ref.WeakReference;
import java.util.*;

public class PeriodicManager {
    private static final Map<String, WeakReference<Periodic>> PERIODICS;

    static {
        PERIODICS = new HashMap<>();
    }

    public static void registerPeriodic(String periodicId, Periodic periodic) {
        PERIODICS.put(periodicId, new WeakReference<>(periodic));
    }

    public static void periodic() {
        PERIODICS.entrySet().removeIf((e) -> e.getValue().get() == null);

        PERIODICS.forEach((name, weak) -> {
            Periodic p = weak.get();
            if(p == null) throw new IllegalStateException("Cannot have a null periodic!");
            p.periodic();
        });
    }
}
