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
    private final SparkMax stagingMotor;
    
    /*
     Shoot fuel by spinning a flywheel
     * The flywheel should keep spinning even after the motor doesn't have power
     */
    
    public Shooter() {
        // motor that runs the shooter
        shooterMotor = new SparkMax(Constants.Shooter.shooterCANId, MotorType.kBrushless);
        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(Constants.Shooter.motorCurrentLimits);
        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, null);
        stagingMotor = new SparkMax(Constants.Shooter.stagingCANId, MotorType.kBrushless); 
        SparkMaxConfig stagingConfig = new SparkMaxConfig();
        stagingConfig
            .smartCurrentLimit(Constants.Shooter.motorCurrentLimits)
            .idleMode(IdleMode.kBrake);
        stagingMotor.configure(stagingConfig, ResetMode.kResetSafeParameters, null);
        feedForward = new SimpleMotorFeedforward(Constants.Shooter.ks, Constants.Shooter.kv);
    }

    public AngularVelocity getShooterSpeed(){
        return AngularVelocity.ofBaseUnits(shooterMotor.getEncoder().getVelocity(), Units.RPM);
    }

    public boolean isShooterReady(){
        return getShooterSpeed().gte(Constants.Shooter.thresholdSpeed);
    }

    private void setShooterSpeed(AngularVelocity speed){
        double voltage = feedForward.calculateWithVelocities(
            getShooterSpeed().in(Units.RPM), 
            speed.in(Units.RPM)
        );
        shooterMotor.setVoltage(voltage);
    }

    private double getCurrentShooterVoltage() {
        return shooterMotor.getBusVoltage();
    }

    public Command stopShooterCommand(){
        return this.runOnce(()->setShooterSpeed(AngularVelocity.ofBaseUnits(0, Units.RPM)));
    }

    public AngularVelocity getStagingSpeed(){
        return AngularVelocity.ofBaseUnits(stagingMotor.getEncoder().getVelocity(), Units.RPM);
    }

    private void setStagingSpeed(AngularVelocity speed){
        double voltage = feedForward.calculateWithVelocities(
            getStagingSpeed().in(Units.RPM), speed.in(Units.RPM));
        stagingMotor.setVoltage(voltage);
    }

    public Command stopStagingCommand(){
        return this.runOnce(()->setStagingSpeed(AngularVelocity.ofBaseUnits(0, Units.RPM)));
    }

    public Command fireCommand(AngularVelocity speed, boolean fuelStaged){
        return this.run(
            // spin the shooter flywheel 
            () -> setShooterSpeed(speed)).until(
                
            // keep spinning until its ready
            ()->isShooterReady()).andThen(
            
            // keep voltage a constant
            () -> shooterMotor.setVoltage(getCurrentShooterVoltage())).alongWith(

            this.runEnd(
            // set staging motor speed
            () -> {
                setStagingSpeed(speed);
            },
            // once interrupted, stop both motors
            () -> {
                stopStagingCommand();
                stopShooterCommand();
            })).onlyIf(()-> fuelStaged);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("motor speed (rotations per minute)", () -> shooterMotor.getEncoder().getVelocity(), null);
    }
}
