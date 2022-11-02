package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IndexerConstants;

public class Indexer {

    public TalonSRX IndexerTop;
    public TalonSRX IndexerBottom;
    private boolean isRunning;

    public Indexer() {
        IndexerTop = new TalonSRX(IndexerConstants.kIndexerTopID);
        IndexerBottom = new TalonSRX(IndexerConstants.kIndexerBottomID);
        IndexerTop.setInverted(true);
        IndexerBottom.setInverted(false);
        
        // IndexerBottom.follow(IndexerTop);
    }


    // public void setVelocity(double topVelocity, double bottomVelocity) {
    //     IndexerTop.set(ControlMode.Velocity, topVelocity);
    //     IndexerBottom.set(ControlMode.Velocity, -bottomVelocity);
    // }

    // public void setVelocityZero() {
    //     IndexerTop.set(ControlMode.Velocity, 0);
    //     IndexerBottom.set(ControlMode.Velocity, 0);
    // }

    public void setPercent(double topPercent, double bottomPercent) {
        IndexerTop.set(ControlMode.PercentOutput, topPercent);
        IndexerBottom.set(ControlMode.PercentOutput, bottomPercent);
    }

    public void setPercentZero() {
        IndexerTop.set(ControlMode.PercentOutput, 0);
        IndexerBottom.set(ControlMode.PercentOutput, 0);
    }

    public void toggleIndexer() {
        SmartDashboard.putBoolean("square button pressed", true);
        if (isRunning) {
            setPercentZero();
            isRunning = false;
        } else {
            setPercent(-0.9, 0.45);
            isRunning = true;
        }
    }

    // public void manualControl(double percent) {
    //     if (!isRunning) {
    //         setPercent(percent);
    //     }
    // }

    public void init() {
        IndexerBottom.set(ControlMode.PercentOutput, 0);
        IndexerTop.set(ControlMode.PercentOutput, 0);
        isRunning = false;
    }

    public void reportToSmartDashboard() {
        
    }

    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Turret");

        tab.addNumber("Top Position", IndexerTop::getSelectedSensorPosition)
            .withPosition(1, 1);
        tab.addNumber("Bottom Position", IndexerBottom::getSelectedSensorPosition)
            .withPosition(1, 2);
        tab.addNumber("Top Percent", IndexerTop::getMotorOutputPercent)
            .withPosition(2, 1);
        tab.addNumber("Bottom Percent", IndexerBottom::getMotorOutputPercent)
            .withPosition(2, 2);
        tab.addNumber("Top Velocity", IndexerTop::getSelectedSensorVelocity)
            .withPosition(3, 1);
        tab.addNumber("Bottom Velocity", IndexerBottom::getSelectedSensorVelocity)
            .withPosition(3, 2);
        tab.addNumber("Top Voltage", IndexerTop::getMotorOutputVoltage)
            .withPosition(4, 1);
        tab.addNumber("Bottom Voltage", IndexerBottom::getMotorOutputVoltage)
            .withPosition(4, 2);

        tab.addBoolean("Is Running", () -> isRunning);
    }
}
