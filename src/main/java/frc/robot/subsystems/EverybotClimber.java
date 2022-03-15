package frc.robot.everybot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EverybotConstants;

public class EverybotClimber {
    public static VictorSPX climberMaster;
    public static VictorSPX climberSlave;

    public static void setUpClimber() {
        climberMaster = new VictorSPX(DriveConstants.kClimbMasterTalonID);
        climberSlave = new VictorSPX(DriveConstants.kClimbSlaveTalonID);

        climberSlave.follow(climberMaster);

        climberSlave.setInverted(InvertType.FollowMaster);
    }

    public static void setPower(double power) {
        climberMaster.set(ControlMode.PercentOutput, power);
    }

    public static void climberUp() {
        if (climberMaster.getSelectedSensorPosition() < EverybotConstants.kEverybotClimberHigh) {
            climberMaster.set(ControlMode.PercentOutput, EverybotConstants.kEverybotClimberUp);
        }
        else {
            climberMaster.set(ControlMode.PercentOutput, 0.05);
        }
    }

    public static void climberDown() {
        if (climberMaster.getSelectedSensorPosition() > EverybotConstants.kEverybotClimberLow) {
            climberMaster.set(ControlMode.PercentOutput, EverybotConstants.kEverybotClimberDown);
        }
        else {
            climberMaster.set(ControlMode.PercentOutput, -0.05);
        }
    }
}
