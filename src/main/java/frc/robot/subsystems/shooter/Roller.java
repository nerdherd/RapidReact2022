package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RollerConstants;

public class Roller {
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
    private TalonSRX roller;
    private boolean isRunning;

    public Roller() {
        roller = new TalonSRX(RollerConstants.kRollerID);
        roller.setInverted(true);
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

    public void toggleRoller(double percent) {
        SmartDashboard.putBoolean("roller button pressed", true);
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

    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Roller");
        tab.addBoolean("Roller Running", () -> isRunning);
        tab.addNumber("Position", roller::getSelectedSensorPosition);
        tab.addNumber("Percent", roller::getMotorOutputPercent);
        tab.addNumber("Velocity", roller::getSelectedSensorVelocity);
        tab.addNumber("Voltage", roller::getMotorOutputVoltage);
    }

    public void init() {
        roller.set(ControlMode.PercentOutput, 0);
        isRunning = false;
    }
}
