package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Shooter extends SubsystemBase {
    private final SparkMax shooterMotor;

    public Shooter() {
       // motor that runs the shooter
       shooterMotor = new SparkMax(Constants.Shooter.motorChannel, Constants.Shooter.motorType);
    }

    private void runShooter() {
        shooterMotor.set(Constants.Shooter.maxSpeed);
    }

    private void stopShooter() {
        shooterMotor.set(Constants.Shooter.stopSpeed);
    }

public Command shootFuelCommand() 
{
    return runOnce(()->this.runShooter());
}

public Command stopShooterCommand() 
{
    return runOnce(() -> this.stopShooter());
}


@Override
public void initSendable(SendableBuilder builder){
    SmartDashboard.putNumber("motorSpeed (rpm)", shooterMotor.getEncoder().getVelocity());
}
}
