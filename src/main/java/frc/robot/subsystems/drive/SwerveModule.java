package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConfig;

public class SwerveModule {
    private final PIDController pid;

    private final TalonFX driveMotor, turnMotor;
    private final AbsoluteEncoder turnEncoder;
    private final RelativeEncoder driveEncoder;

    private final State moduleState;

    public SwerveModule(int driveCANId, int turnCANId) {
        pid = new PIDController(1,0,0);
        driveMotor = new TalonFX(driveCANId);
        turnMotor = new TalonFX(turnCANId);
    }

    //Radians
    public double getTurnPosition() {
        return this.turnEncoder.getPosition();
    }
    public Rotation2d getRot2d() {
        return Rotation2d.fromRadians(this.getTurnPosition());
    }
    // speed m/s
    public double getDriveSpeed() {
        return this.driveEncoder.getVelocity();
    }
    
    public State getModuleState() {
        return this.moduleState;
    }

    public double getMaxSpeed() {
        return this.driveMotor.getMaxSpeed();
    }

    public void requestState(SwerveModuleState requested) {
        requested.optimize(getRot2d());
        requested.cosineScale(getRot2d());
        setDriveSpeed(requested.speedMetersPerSecond);
        setTurnPos(requested.angle.getRadians());
    }

    public void populateContainer(String prefix, SysIdRoutineContainer container) {
        container.addDriveMotor(prefix + "D", this.driveMotor)
                .addTurnMotor(prefix + "T", this.turnMotor, this.turnEncoder);
    }

    public void test() {
        //this.driveMotor.setVoltage(1);
        //this.turnMotor.setVoltage(1);
        this.setTurnPos(0);
    }

    //m/s
    private void setDriveSpeed(double speed) {
        double volts = this.context.calculate(speed, getDriveSpeed(), this.driveEncoder.getVelocity(), false);
        this.driveMotor.setVoltage(volts);
    }
    private void setTurnPos(double rads) {
        double volts = this.context.calculate(
            rads, getTurnPosition(), this.turnEncoder.getVelocity(), true
        );
        this.turnMotor.setVoltage(volts);
    }

    private Encoder getDriveEncoder() {
        return driveEncoder;
    }

    public record State(SwerveModule module) {
        public SwerveModulePosition asPosition() {
            return new SwerveModulePosition(
                    module.getDriveEncoder().getPosition(),
                    module.getRot2d()
            );
        }
        public SwerveModuleState asState() {
            return new SwerveModuleState(
                    module.getDriveSpeed(),
                    module.getRot2d()
            );
        }
    }
}
