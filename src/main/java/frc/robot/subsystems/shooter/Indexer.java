package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IndexerConstants;

public class Indexer {

    public TalonFX IndexerTop;
    public TalonFX IndexerBottom;
    private boolean isRunning;

    public Indexer() {
        IndexerTop = new TalonFX(IndexerConstants.kIndexerTopID);
        IndexerBottom = new TalonFX(IndexerConstants.kIndexerBottomID);
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
            setPercent(0.9, 0.45);
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
}
