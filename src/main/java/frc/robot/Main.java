// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

import java.util.function.Function;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot call.
 */
//Dw, I know what I'm doing ;)
public final class Main {

    private Main() {}

   /**
    * Main initialization method. Do not perform any initialization here.
    * <p>
    * If you change your main Robot class (name), change the parameter type.
    */
    public static void main(String... args) {
        
    }

    private static <T extends RobotBase> void startRobotWithHardware(Function<Hardware, T> factory) {
        RobotBase.startRobot(
                () -> {
                    Hardware hardware = new Hardware();
                    HardwareImpl.register(hardware);
                    return factory.apply(hardware);
                }
        );
    }

}
