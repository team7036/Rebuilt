// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.LazyCachedFunction;
import frc.robot.util.LazyValue;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public final class Constants {
    public static class SwerveConfig {
        public int driveCANId;
        public int turnCANId;
        public SwerveConfig(int driveCANId, int turnCANId){
            this.driveCANId = driveCANId;
            this.turnCANId = turnCANId;
        }
    }

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}
