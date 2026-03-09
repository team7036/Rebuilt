package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
    private final SparkMax shooterMotor;
    private final SimpleMotorFeedforward feedForward;
    
    public Shooter() {
        // motor that runs the shooter
        shooterMotor = new SparkMax(Constants.Shooter.CANId, MotorType.kBrushless);
        feedForward = new SimpleMotorFeedforward(Constants.Shooter.ks, Constants.Shooter.kv);
    }

    public AngularVelocity getFlywheelSpeed(){
        return AngularVelocity.ofBaseUnits(shooterMotor.getEncoder().getVelocity(), Units.RPM);
    }

    public boolean isFlywheelReady(){
        return getFlywheelSpeed().gte(Constants.Shooter.thresholdSpeed);
    }

    private void setSpeed(AngularVelocity speed){
        double voltage = feedForward.calculateWithVelocities(
            getFlywheelSpeed().in(Units.RPM), 
            speed.in(Units.RPM)
        );
        shooterMotor.setVoltage(voltage);
    }

    public Command setSpeedCommand(AngularVelocity speed){
        return this.run(()->setSpeed(speed))
            .until(()->isFlywheelReady());
    }

    public Command stopCommand(){
        return this.runOnce(()->setSpeed(AngularVelocity.ofBaseUnits(0, Units.RPM)));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("motor speed (rotations per minute)", () -> shooterMotor.getEncoder().getVelocity(), null);
    }
}
