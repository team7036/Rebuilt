package frc.robot.hardware.impl.gyro;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.hardware.Gyro;
import frc.robot.hardware.Hardware;

public record WPILIBGyro(ADXRS450_Gyro gyro) implements Gyro {
    @Override
    public double getAngle() {
        return Units.degreesToRadians(this.gyro.getAngle());
    }

    @Override
    public void reset() {
        this.gyro.reset();
    }

    public static WPILIBGyro of(Hardware hardware, int deviceId) {
        ADXRS450_Gyro gyro = new ADXRS450_Gyro();
        gyro.calibrate();
        return new WPILIBGyro(gyro);
    }
}
