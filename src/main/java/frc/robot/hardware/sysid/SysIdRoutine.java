package frc.robot.hardware.sysid;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class SysIdRoutine {
    private final List<SysIdRoutineContainer.BuiltSubRoutine> subRoutines;

    public SysIdRoutine() {
        this.subRoutines = new ArrayList<>();
    }

    public SysIdRoutine addRoutine(SysIdRoutineContainer.BuiltSubRoutine routine) {
        this.subRoutines.add(routine);
        return this;
    }

    public Command run() {
        Command[] commands = this.subRoutines.stream().map(SysIdRoutineContainer.BuiltSubRoutine::run)
                .map(Supplier::get).toArray(Command[]::new);
        return Commands.sequence(commands);
    }
}
