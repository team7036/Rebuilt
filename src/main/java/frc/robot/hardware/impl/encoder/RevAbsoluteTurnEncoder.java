package frc.robot.hardware.impl.encoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAbsoluteEncoder;
import frc.robot.hardware.Encoder;
import frc.robot.hardware.Hardware;

public record RevAbsoluteTurnEncoder(AbsoluteEncoder encoder, double offsetRad) implements Encoder {

    @Override
    public double getPosition() {
        // rotations [0,1) → radians
        return encoder.getPosition() * 2.0 * Math.PI - offsetRad;
    }

    @Override
    public double getVelocity() {
        // rotations/sec → rad/sec
        return encoder.getVelocity() * 2.0 * Math.PI;
    }

    public static RevAbsoluteTurnEncoder of(
            Hardware hardware,
            int canId,
            double offsetRad
    ) {
        //noinspection resource
        SparkMax spark = new SparkMax(canId, SparkLowLevel.MotorType.kBrushless);

        AbsoluteEncoder abs =
                spark.getAbsoluteEncoder();

        return new RevAbsoluteTurnEncoder(abs, offsetRad);
    }
}
