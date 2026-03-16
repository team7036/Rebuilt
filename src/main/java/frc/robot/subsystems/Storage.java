package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Storage extends SubsystemBase {
    //Storage motor for raising balls into the shooter.

    private final SparkMax storageMotor;
    private int numberOfBalls = 0;
    private boolean motorup = false;
    private final DigitalInput incomingSensor;
    private final DigitalInput outgoingSensor;


    public Storage(){
    //Storage motor for raising balls into the shooter.
       storageMotor = new SparkMax(Constants.Storage.motorCanId, MotorType.kBrushless);
       incomingSensor = new DigitalInput(0);
       outgoingSensor = new DigitalInput(1);
    }

   private Command motorSpinUp(){
    return this.run(null);

   }

    private Command motorSpinDown(){
    return this.run(null);
   }

    public Command storageraise(){
        if (hasBalls()){
            motorup = true;
            return motorSpinUp();  
        }
        else{
            motorup = false;
            return null;
        }
    }


    public Command storagelower(){
        return motorSpinDown();
    }

    public int getNumberOfBalls(){
        return numberOfBalls;
    }

    public boolean hasBalls() {
        return numberOfBalls > 0;
    }

    public boolean isUp(){
        return motorup;
    }
    

    @Override
    public void initSendable(SendableBuilder builder){
        builder.addIntegerProperty("numBalls", this::getNumberOfBalls, null);
    }

}

