package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
    private final SparkMax shooterMotor;
    private final SimpleMotorFeedforward feedForward;
    private boolean shooterStopTriggered;
    
    /*
     Shoot fuel by spinning a flywheel
     * The flywheel should keep spinning even after the motor doesn't have power
     */
    
    public Shooter() {
        // motor that runs the shooter
        shooterMotor = new SparkMax(Constants.Shooter.shooterCANId, MotorType.kBrushless);
        
        // boolean that is useful for the storage subsystem
        shooterStopTriggered = false;

        // configure the shooter
        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(Constants.Shooter.motorCurrentLimits);
        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, null);

        // initialize the feedforward system
        feedForward = new SimpleMotorFeedforward(Constants.Shooter.ks, Constants.Shooter.kv);

        // set the default command
        this.setDefaultCommand(stopShooterCommand());
    }

    public AngularVelocity getShooterSpeed(){
        return AngularVelocity.ofBaseUnits(shooterMotor.getEncoder().getVelocity(), Units.RPM);
    }

    public boolean isShooterReady(){
        return getShooterSpeed().gte(Constants.Shooter.thresholdSpeed);
    }

    private void setShooterSpeed(AngularVelocity speed){
        shooterStopTriggered = false;
        double voltage = feedForward.calculateWithVelocities(
            getShooterSpeed().in(Units.RPM), 
            speed.in(Units.RPM)
        );
        shooterMotor.setVoltage(voltage);
    }

    public boolean shooterBeenStopped() {
        return shooterStopTriggered;
    }

    private double getCurrentShooterVoltage() {
        return shooterMotor.getBusVoltage();
    }

    public Command stopShooterCommand(){
        shooterStopTriggered = true;
        return this.runOnce(()->setShooterSpeed(AngularVelocity.ofBaseUnits(0, Units.RPM)));
    }
    public Command fireCommand(AngularVelocity speed, boolean fuelPresent){
        return this.run(
            // spin the shooter flywheel 
            () -> setShooterSpeed(speed)).until(
                
            // keep spinning until its ready
            ()->isShooterReady()).andThen(
            
            // keep voltage a constant
            () -> shooterMotor.setVoltage(getCurrentShooterVoltage())).onlyIf(()-> fuelPresent);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("motor speed (rotations per minute)", () -> shooterMotor.getEncoder().getVelocity(), null);
    }
}
