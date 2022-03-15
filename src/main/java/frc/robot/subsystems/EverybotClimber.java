package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EverybotConstants;

public class EverybotClimber extends SubsystemBase {
    public VictorSPX climberMaster;
    public VictorSPX climberSlave;

    public EverybotClimber() {
        climberMaster = new VictorSPX(DriveConstants.kClimbMasterTalonID);
        climberSlave = new VictorSPX(DriveConstants.kClimbSlaveTalonID);

        climberSlave.follow(climberMaster);

        climberSlave.setInverted(InvertType.FollowMaster);
    }

    public void setPower(double power) {
        climberMaster.set(ControlMode.PercentOutput, power);
    }

    public void climberUp() {
        if (climberMaster.getSelectedSensorPosition() < EverybotConstants.kEverybotClimberHigh) {
            climberMaster.set(ControlMode.PercentOutput, EverybotConstants.kEverybotClimberUp);
        }
        else {
            climberMaster.set(ControlMode.PercentOutput, 0.05);
        }
    }

    public void climberDown() {
        if (climberMaster.getSelectedSensorPosition() > EverybotConstants.kEverybotClimberLow) {
            climberMaster.set(ControlMode.PercentOutput, EverybotConstants.kEverybotClimberDown);
        }
        else {
            climberMaster.set(ControlMode.PercentOutput, -0.05);
        }
    }
}
