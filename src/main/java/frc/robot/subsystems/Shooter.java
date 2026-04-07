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
    private final PIDController pid = new PIDController(0, 0, 0);

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

    private boolean isReady(){
        // Boolean for whether the shooter has reached the desired velocity
        return true;
    }

    private Command idleCommand(){
        return this.runOnce(()->{
            stagingMotor.set(0);
            shootingMotor.set(0);
        });
    }

    public Command runStagingCommand(){
        return this.run(()->stagingMotor.set(-0.2));
    }

    public Command fireFuelCommand(){
        // Command for launching all of the loaded fuel
        return this.run(()->shootingMotor.set(1));
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ShooterSubsystem");
        SmartDashboard.putData("Shooter/fireFuelCommand", fireFuelCommand());
        SmartDashboard.putData("Shooter/runStagingCommand", runStagingCommand());
    }
}
