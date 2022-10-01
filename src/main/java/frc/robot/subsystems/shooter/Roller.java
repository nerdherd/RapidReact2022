package frc.robot.subsystems.shooter;

import javax.swing.SizeRequirements;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RollerConstants;

public class Roller {

    private TalonSRX roller;
    private boolean isRunning;

    public Roller() {
        roller = new TalonSRX(RollerConstants.kRollerID);
        roller.set(ControlMode.PercentOutput, 0);
        isRunning = false;
    }

    // public void setVelocity(double velocity) {
    //     IndexerTop.set(ControlMode.Velocity, velocity);
    // }

    // public void setVelocityZero() {
    //     IndexerTop.set(ControlMode.Velocity, 0);
    // }

    private void setPercent(double percent) {
        roller.set(ControlMode.PercentOutput, percent);
    }

    private void setPercentZero() {
        roller.set(ControlMode.PercentOutput, 0);
    }

    public void toggleRoller(double percent) {
        if (isRunning) {
            setPercentZero();
            isRunning = false;
        } else {
            setPercent(percent);
            isRunning = true;
        }
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putBoolean("Roller Running", isRunning);
        
    }
}
