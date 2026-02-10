package frc.robot.hardware.sysid;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.util.Map;

public record SydIdLogAccessor(SysIdRoutineLog log) {
    public Map<String, Map<String, DoubleLogEntry>> access() {
        try {
            VarHandle handle = MethodHandles.privateLookupIn(
                    SysIdRoutineLog.class,
                    MethodHandles.lookup()
            ).findVarHandle(SysIdRoutineLog.class, "m_logEntries", Map.class);

            //noinspection unchecked
            return (Map<String, Map<String, DoubleLogEntry>>) handle.get(log);
        } catch (Exception e) {
            throw new RuntimeException("Failed to access logs: ", e);
        }
    }
}
