package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FlywheelConstants;

public class Flywheel {

    private TalonFX leftMaster;
    private TalonFX rightFollower;

    public Flywheel() {
        leftMaster = new TalonFX(FlywheelConstants.kLeftFlywheelID);
        rightFollower = new TalonFX(FlywheelConstants.kRightFlywheelID);
        
        leftMaster.setInverted(true);
        rightFollower.setInverted(false);

        rightFollower.follow(leftMaster);
    }

    public void setVelocity(double velocity) {
        leftMaster.set(ControlMode.Velocity, velocity);
    }

    public void setVelocityZero() {
        leftMaster.set(ControlMode.Velocity, 0);
    }

    public void reportToSmartDashboard() {
        
    }
}
