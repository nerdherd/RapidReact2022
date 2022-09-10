package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RollerConstants;

public class Roller {

    private TalonSRX roller;

    public Roller() {
        roller = new TalonSRX(RollerConstants.kRollerID);
    }

    // public void setVelocity(double velocity) {
    //     IndexerTop.set(ControlMode.Velocity, velocity);
    // }

    // public void setVelocityZero() {
    //     IndexerTop.set(ControlMode.Velocity, 0);
    // }

    public void setPercent(double percent) {
        roller.set(ControlMode.PercentOutput, percent);
    }

    public void setPercentZero() {
        roller.set(ControlMode.PercentOutput, 0);
    }

    public void reportToSmartDashboard() {
        
    }
}
