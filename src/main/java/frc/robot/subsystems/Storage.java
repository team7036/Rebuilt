package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


public class Storage extends SubsystemBase {


    //Storage motor for raising balls into the shooter.

    private final SparkMax storageMotor;
    private int numberOfBalls = 0;


    public Storage(){
        //Storage motor for raising balls into the shooter.
       storageMotor = new SparkMax(0, MotorType.kBrushless);
    }

   private Command motorSpinUp(){
    return this.run(null);
   }

    private Command motorSpinDown(){

    return this.run(null);

   }

    public Command storageraise(){
        if (numberOfBalls > 0){
            return runOnce(null);
        }
        else{
            return null;
        }
    }


    public Command storagelower(){
        return runOnce(null);
    }

    public int getNumberOfBalls(){
        return numberOfBalls;
    }

}

