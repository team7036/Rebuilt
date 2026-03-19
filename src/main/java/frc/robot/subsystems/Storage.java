package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Storage extends SubsystemBase {
    // Storage motor for raising balls into the shooter.

    private final SparkMax feedMotor;
    public int numberOfBalls = 0;
    public boolean hasFuel;
    private final DigitalInput incomingSensor;
    // private final DigitalInput outgoingSensor;

    public Storage() {
        // Storage motor for raising balls into the shooter.
        feedMotor = new SparkMax(Constants.Storage.motorCanId, MotorType.kBrushless);
        incomingSensor = new DigitalInput(0);
        // outgoingSensor = new DigitalInput(1);
        hasFuel = false;
        this.setDefaultCommand(stopFeedCommand());
    }

    public Command stopFeedCommand() {
        return this.run(() -> feedMotor.set(0));
    }

   public Command startFeedCommand(){
    return this.run(()->feedMotor.set(Constants.Storage.feedSpeed)).onlyIf(()->storageHasFuel(hasFuel));
   }

   public boolean storageHasFuel(boolean shooterBeenStopped){
    if (incomingSensor.get()) {
        hasFuel = true;
    }
    else {
        if (shooterBeenStopped) {
            hasFuel = false;
        }
    }
    return hasFuel;
   }

//    @Override
//    public void periodic(){
//     if (incomingSensor.get()){
//         numberOfBalls++;
//     } else if (outgoingSensor.get()){
//         numberOfBalls--;
//     }
//    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addIntegerProperty("numBalls", ()->numberOfBalls, null);
    }

}
