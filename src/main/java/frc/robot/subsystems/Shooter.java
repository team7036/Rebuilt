package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private final SparkMax shootingMotor;
    private final SparkMaxConfig shootingConfig;
    private final SparkMax shootingFollowerMotor;
    private final SparkMaxConfig shootingFollowerConfig;
    private final SparkMax stagingMotor;
    private final SparkMaxConfig stagingConfig;
    private final DigitalInput fuelSensor = new DigitalInput(Constants.IDs.DIO.ShooterFuelSensor);
    private double targetSpeed;

    public Shooter(){
        shootingMotor = new SparkMax(Constants.IDs.CAN.ShooterLeader, MotorType.kBrushless);
        shootingConfig = new SparkMaxConfig();
        shootingFollowerMotor = new SparkMax(Constants.IDs.CAN.ShooterFollower, MotorType.kBrushless);
        shootingFollowerConfig = new SparkMaxConfig();
        stagingMotor = new SparkMax(Constants.IDs.CAN.ShooterStaging, MotorType.kBrushless);
        stagingConfig = new SparkMaxConfig();
        configureMotors();
        this.setDefaultCommand(idleCommand());
    }

    private void configureMotors(){
        // Shooting Motor
        shootingConfig.idleMode(IdleMode.kCoast);
        shootingConfig.smartCurrentLimit(40);
        shootingMotor.configure(shootingConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // Follower Motor
        shootingFollowerConfig
            .apply(shootingConfig)
            .follow(shootingMotor, true);
        shootingFollowerMotor.configure(shootingFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // Staging Motor
        stagingConfig.idleMode(IdleMode.kBrake);
        stagingConfig.smartCurrentLimit(40);
        stagingMotor.configure(stagingConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private double getFlyWheelSpeed(){
        return shootingMotor.getEncoder().getVelocity();
    }

    private boolean isReadyToFire(){
        // The Maximum achievable RPM of the NEO depends on the supply
        // voltage recieved from the power supply.

        // 12.2V -> 5080 RPM
        // 11.8V -> 5000 RPM

        // TODO Get this to be more dynamic and change with the distance
        return getFlyWheelSpeed() > 4500;
    }

    public boolean hasFuel(){
        return !fuelSensor.get();
    }

    private void spinupFlywheel(){
        shootingMotor.set(1);
    }

    private void stopFlywheel(){
        shootingMotor.set(0);
    }

    private void stopStaging(){
        stagingMotor.set(0);
    }

    private void startStaging(){
        stagingMotor.set(Constants.Shooter.STAGING_SPEED);
    }

    private Command idleCommand(){
        return this.run(()->{
            stopFlywheel();
            stopStaging();
        });
    }

    public Command fireFuelCommand(){
        // Command for launching all of the loaded fuel
        return this.run(this::spinupFlywheel).until(this::isReadyToFire).andThen(this::startStaging);
    }

    public Command intakeFuelCommand(){
        return this.run(this::startStaging).onlyWhile(()->!this.hasFuel());
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ShooterSubsystem");
        builder.addDoubleProperty("flywheelSpeed", this::getFlyWheelSpeed, null);
        builder.addDoubleProperty("busVoltage", this.shootingMotor::getBusVoltage, null);
        builder.addBooleanProperty("hasFuel", this::hasFuel, null);
    }
}
