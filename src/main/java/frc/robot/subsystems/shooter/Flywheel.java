package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.jsontype.impl.LaissezFaireSubTypeValidator;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FlywheelConstants;

public class Flywheel {

    private TalonFX leftMaster;
    private TalonFX rightFollower;
    private boolean isRunning;

    public Flywheel() {
        leftMaster = new TalonFX(FlywheelConstants.kLeftFlywheelID);
        rightFollower = new TalonFX(FlywheelConstants.kRightFlywheelID);
        
        leftMaster.setInverted(false);
        rightFollower.setInverted(true);

        rightFollower.follow(leftMaster);
        isRunning = false;
        leftMaster.set(ControlMode.PercentOutput, 0);
    }

    public void setVelocity(double velocity) {
        leftMaster.set(ControlMode.Velocity, velocity);
    }

    public void setVelocityZero() {
        leftMaster.set(ControlMode.Velocity, 0);
    }

    public void setPercent(double percent) {
        leftMaster.set(ControlMode.PercentOutput, percent);
    }

    public void setPercentZero() {
        leftMaster.set(ControlMode.PercentOutput, 0);
    }

    public void toggleFlywheel() {
        if (!isRunning) {
            setPercent(FlywheelConstants.kFlywheelPercent);
            isRunning = true;
        } else {
            setPercentZero();
            isRunning = false;
        }
    }

    public void reportToSmartDashboard() {
        
    }
}
