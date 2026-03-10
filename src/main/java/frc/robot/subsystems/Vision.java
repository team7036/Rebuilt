package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private static boolean sendableInitialized;

    public Vision() {
        super("Vision");
    }


    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Vision");
        SmartDashboard.putData("vision", this);
        this.populate(builder);
        super.initSendable(builder);
    }

    private void populate(SendableBuilder builder) {

    }
    
}
