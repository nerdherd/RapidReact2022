package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class PassiveClimber extends SubsystemBase {
    
    public TalonFX climberLeft;
    public TalonFX climberRight;

    public PassiveClimber() {
        climberLeft = new TalonFX(ClimberConstants.kClimberLeftID);
        climberRight = new TalonFX(ClimberConstants.kClimberRightID);

        // invert one here

        // config tuning params in slot 0
        climberLeft.config_kP(0, ClimberConstants.kP);
        climberLeft.config_kI(0, ClimberConstants.kI);
        climberLeft.config_kD(0, ClimberConstants.kD);
        climberLeft.config_kF(0, ClimberConstants.kF);

        climberLeft.config_kP(0, ClimberConstants.kP);
        climberLeft.config_kI(0, ClimberConstants.kI);
        climberLeft.config_kD(0, ClimberConstants.kD);
        climberLeft.config_kF(0, ClimberConstants.kF);

        climberLeft.configMotionCruiseVelocity(ClimberConstants.kCruiseVelocity);
        climberLeft.configMotionAcceleration(ClimberConstants.kMotionAcceleration);

        climberRight.configMotionCruiseVelocity(ClimberConstants.kCruiseVelocity);
        climberRight.configMotionAcceleration(ClimberConstants.kMotionAcceleration);
    }

    public void setPositionLeft(double ticks) {
        climberLeft.set(ControlMode.MotionMagic, ticks);
    }

    public void setPositionRight(double ticks) {
        climberRight.set(ControlMode.MotionMagic, ticks);
    }

    public void setPositionBoth(double ticks) {
        climberRight.set(ControlMode.MotionMagic, ticks);
    }

    public void init() {
        climberLeft.set(ControlMode.PercentOutput, 0);
        climberRight.set(ControlMode.PercentOutput, 0);
    }    
}
