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
        
        
        // IndexerBottom.follow(IndexerTop);
    }

    // public void setVelocity(double velocity) {
    //     IndexerTop.set(ControlMode.Velocity, velocity);
    // }

    // public void setVelocityZero() {
    //     IndexerTop.set(ControlMode.Velocity, 0);
    // }

    public void setPercent(double percent) {
        IndexerTop.set(ControlMode.PercentOutput, percent/2);
        IndexerBottom.set(ControlMode.PercentOutput, percent);
    }

    public void setPercentZero() {
        IndexerTop.set(ControlMode.PercentOutput, 0);
        IndexerBottom.set(ControlMode.PercentOutput, 0);
    }

    public void toggleIndexer() {
        if (isRunning) {
            setPercentZero();
            isRunning = false;
        } else {
            setPercent(IndexerConstants.kIndexerPercent);
            isRunning = true;
        }
    }

    public void init() {
        IndexerBottom.set(ControlMode.PercentOutput, 0);
        IndexerTop.set(ControlMode.PercentOutput, 0);
        isRunning = false;
    }

    public void reportToSmartDashboard() {
        
    }
}
