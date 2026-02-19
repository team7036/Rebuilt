package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final SparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final ArmFeedforward angleFeedforward;
    private final PIDController anglePID;
    private final SparkMax intakeMotor;

    public Intake(){
        angleMotor = new SparkMax(0, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        angleFeedforward = new ArmFeedforward(0, 0, 0);
        anglePID = new PIDController(0, 0, 0);
        intakeMotor = new SparkMax(0, MotorType.kBrushless);
    }

    private double getAngle(){
        return angleEncoder.getPosition();
    }

    private void setAngle(double radians){
        angleMotor.setVoltage(
            angleFeedforward.calculate(getAngle(), 0) +
            anglePID.calculate(radians, getAngle())
        );
    }

    public Command intakeCommand(){
        /*
         * 1. Lower the 
         */
        return this.run(null);
    }
}
