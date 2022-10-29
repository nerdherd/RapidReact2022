package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FlywheelConstants;

public class Flywheel {
    // public TalonFX leftMaster;
    // public TalonFX rightFollower;

    public TalonFX flywheel;
    public TalonSRX feeder;
    private boolean isRunning;

    public Flywheel() {
        // leftMaster = new TalonFX(FlywheelConstants.kLeftFlywheelID);
        // rightFollower = new TalonFX(FlywheelConstants.kRightFlywheelID);

        
        // leftMaster.setInverted(false);
        // rightFollower.setInverted(true);

        // rightFollower.follow(leftMaster);
        flywheel = new TalonFX(FlywheelConstants.kFlywheelID);
        feeder = new TalonSRX(FlywheelConstants.kFeederID);

        flywheel.setInverted(false);
        feeder.setInverted(true);
    }

    // public void setVelocity(double velocity) {
    //     flywheel.set(ControlMode.Velocity, velocity);
    // }

    // public void setVelocityZero() {
    //     flywheel.set(ControlMode.Velocity, 0);
    // }

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
}
