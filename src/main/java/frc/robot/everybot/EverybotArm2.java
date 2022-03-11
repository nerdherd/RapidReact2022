package frc.robot.everybot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.RobotMap;
import frc.robot.constants.EverybotConstants;

public class EverybotArm2 {
    public static TalonFX arm;

    public static void setUpEArm() {
        arm = new TalonFX(RobotMap.kEverybotArm);
    }

    public EverybotArm2() {
        arm.setInverted(false);
    }

    public static void Arm2Up() {
        if (arm.getSelectedSensorPosition() < (EverybotConstants.kHighAngle-100)) {
            arm.set(ControlMode.PercentOutput, EverybotConstants.kEverybotUpVoltage);
        }
        else {
            arm.set(ControlMode.PercentOutput, 0.08);
        }
    }

    public static void Arm2Down() {
        if (arm.getSelectedSensorPosition() > EverybotConstants.kGravityAngle) {
            arm.set(ControlMode.PercentOutput, EverybotConstants.kEverybotDownVoltage);
        }
        else if (Math.abs(arm.getSelectedSensorPosition() - EverybotConstants.kLowAngle) < EverybotConstants.kLowAngleThreshold) {
            arm.set(ControlMode.PercentOutput, -0.05);
        }
        else {
            arm.set(ControlMode.PercentOutput, 0);
        }
    }
}
