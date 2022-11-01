package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class PassiveClimber extends SubsystemBase {
    
    public TalonFX climberLeft;
    public TalonFX climberRight;

    public PassiveClimber() {
        //climberLeft = new TalonFX(ClimberConstants.kClimberLeftID);
        climberRight = new TalonFX(19);
    }

    public void move (double currentJoystickOutput) {
        double climberTicks = climberRight.getSelectedSensorPosition();

        // Soft limit

        // if (currentJoystickOutput > ClimberConstants.kClimberDeadband) {
        //     if (climberTicks < ClimberConstants.kMaxTicks) {
        //         climberRight.set(ControlMode.PercentOutput, 
        //             (ClimberConstants.kF + (currentJoystickOutput * ClimberConstants.kJoystickMultiplier)));
        //     }
        // } else if (currentJoystickOutput < -ClimberConstants.kClimberDeadband) {
        //     if (climberTicks > ClimberConstants.kMinTicks) {
        //         climberRight.set(ControlMode.PercentOutput, 
        //             (ClimberConstants.kF + (currentJoystickOutput * ClimberConstants.kJoystickMultiplier)));
        //     }
        // } else {
        //     climberRight.set(ControlMode.PercentOutput, ClimberConstants.kF);
        // }

        // No soft limit

        if (currentJoystickOutput > ClimberConstants.kClimberDeadband) {
            climberRight.set(ControlMode.PercentOutput, 
                (ClimberConstants.kF + (currentJoystickOutput * ClimberConstants.kJoystickMultiplier)));
        } else if (currentJoystickOutput < -ClimberConstants.kClimberDeadband) {
            climberRight.set(ControlMode.PercentOutput, 
                (ClimberConstants.kF + (currentJoystickOutput * ClimberConstants.kJoystickMultiplier)));
        } else {
            climberRight.set(ControlMode.PercentOutput, ClimberConstants.kF);
        }
    }

    //     // invert one here (choose one)
    //     // climberLeft.setInverted(false);
    //     // climberRight.setInverted(true);

    //     // climberLeft.setInverted(true);
    //     // climberRight.setInverted(false);

    //     // config tuning params in slot 0
    //     climberLeft.config_kP(0, ClimberConstants.kP);
    //     climberLeft.config_kI(0, ClimberConstants.kI);
    //     climberLeft.config_kD(0, ClimberConstants.kD);

    //     climberRight.config_kP(0, ClimberConstants.kP);
    //     climberRight.config_kI(0, ClimberConstants.kI);
    //     climberRight.config_kD(0, ClimberConstants.kD);

    //     climberLeft.configMotionCruiseVelocity(ClimberConstants.kCruiseVelocity);
    //     climberLeft.configMotionAcceleration(ClimberConstants.kMotionAcceleration);

    //     climberRight.configMotionCruiseVelocity(ClimberConstants.kCruiseVelocity);
    //     climberRight.configMotionAcceleration(ClimberConstants.kMotionAcceleration);
    // }

    // /* Percent Output Control */

    // public void setPower(double power) {
    //     setPowerLeft(power);
    //     setPowerRight(power);
    // }

    // public void setPowerLeft(double power) {
    //     climberLeft.set(ControlMode.PercentOutput, power);
    // }

    // public void setPowerRight(double power) {
    //     climberRight.set(ControlMode.PercentOutput, power);
    // }

    public void setPowerZero() {
        climberRight.set(ControlMode.PercentOutput, 0.0);
    }

    // /* Position Control */

    // public void setPositionLeft(double ticks) {
    //     climberLeft.set(ControlMode.MotionMagic, ticks);
    // }

    // public void setPositionRight(double ticks) {
    //     climberRight.set(ControlMode.MotionMagic, ticks);
    // }


    // public void setPositionBoth(double ticks) {
    //     climberRight.set(ControlMode.MotionMagic, ticks);
    // }

    public void init() {
        setPowerZero();
    }    

    // public void reportToSmartDashboard() {

    // }

    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Climber");
        tab.addNumber("Position", climberRight::getSelectedSensorPosition);
        tab.addNumber("Percent", climberRight::getMotorOutputPercent);
        tab.addNumber("Velocity", climberRight::getSelectedSensorVelocity);
        tab.addNumber("Voltage", climberRight::getMotorOutputVoltage);
    }
}
