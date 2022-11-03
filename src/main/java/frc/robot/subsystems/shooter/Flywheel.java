package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FlywheelConstants;

public class Flywheel {
    // public TalonFX leftMaster;
    // public TalonFX rightFollower;

    public TalonFX flywheel;
    public TalonSRX feeder;
    private boolean isRunning;

    public Flywheel() {

        // rightFollower.follow(leftMaster);
        flywheel = new TalonFX(FlywheelConstants.kFlywheelID);
        feeder = new TalonSRX(FlywheelConstants.kFeederID);

        flywheel.setInverted(false);
        feeder.setInverted(true);

        
        flywheel.config_kP(0, FlywheelConstants.kFlywheelP);
        flywheel.config_kI(0, FlywheelConstants.kFlywheelI);
        flywheel.config_kD(0, FlywheelConstants.kFlywheelD);
        flywheel.config_kF(0, FlywheelConstants.kFlywheelFF);
        
        feeder.config_kP(0, FlywheelConstants.kFeederP);
        feeder.config_kI(0, FlywheelConstants.kFeederI);
        feeder.config_kD(0, FlywheelConstants.kFeederD);
        feeder.config_kF(0, FlywheelConstants.kFeederFF);
    }

    public void setVelocity(double flywheelVelocity, double feederVelocity) {
        flywheel.set(ControlMode.Velocity, flywheelVelocity);
        feeder.set(ControlMode.Velocity, feederVelocity);
    }
    
    public void setFlywheelVelocity(double velocity) {
        flywheel.set(ControlMode.Velocity, velocity);
    }

    public void setFeederPercent(double percent) {
        feeder.set(ControlMode.PercentOutput, percent);
    }

    public void setPercent(double flywheelPercent, double feederPercent) {
        flywheel.set(ControlMode.PercentOutput, flywheelPercent);
        feeder.set(ControlMode.PercentOutput, feederPercent);
    }

    public void setPercentZero() {
        flywheel.set(ControlMode.PercentOutput, 0);
        feeder.set(ControlMode.PercentOutput, 0);
    }

    // public void flywheelInnerTarmac() {
    //     flywheelVelocity
    // }

    public void toggleFlywheel() {
        if (!isRunning) {
            setPercent(0.6, -0.4); // flywheel, feeder
            isRunning = true;
        } else {
            setPercentZero();
            isRunning = false;
        }
    }

    public void manualFlywheel() {
        if (!isRunning) {
            setPercent(0.1, 0);
        }
    }

    public void init() {
        SmartDashboard.putNumber("flywheelVelocity", FlywheelConstants.kFlywheelPercent);
        isRunning = false;
        flywheel.set(ControlMode.PercentOutput, 0);
    }

    public void reportToSmartDashboard() { }

    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Flywheel");
        tab.addNumber("Velocity", flywheel::getSelectedSensorVelocity);
        tab.addNumber("Voltage", flywheel::getMotorOutputVoltage);
    }
}
