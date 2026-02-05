// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.hardware.Hardware;
import frc.robot.hardware.impl.HardwareImpl;

import java.util.function.Function;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot call.
 */
//Dw, I know what I'm doing ;)
public final class Main {
    private static final Hardware ROBOT_HARDWARE = new Hardware();

    private Main() {}

   /**
    * Main initialization method. Do not perform any initialization here.
    * <p>
    * If you change your main Robot class (name), change the parameter type.
    */
    public static void main(String... args) {
        HardwareImpl.register(ROBOT_HARDWARE);
        startRobotWithHardware(Robot::new);
    }

    private static <T extends RobotBase> void startRobotWithHardware(Function<Hardware, T> factory) {
        RobotBase.startRobot(
                () -> factory.apply(Main.ROBOT_HARDWARE)
        );
    }

}
