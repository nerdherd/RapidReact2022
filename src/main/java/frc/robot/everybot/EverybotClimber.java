package frc.robot.everybot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotMap;

public class EverybotClimber {
    public static TalonSRX climberMaster;
    public static TalonSRX climberSlave;

    public static void setUpClimber() {
        climberMaster = new TalonSRX(RobotMap.kClimbMasterTalonID);
        climberSlave = new TalonSRX(RobotMap.kClimbSlaveTalonID);

        climberSlave.follow(climberMaster);

        climberSlave.setInverted(InvertType.FollowMaster);
    }

    public static void raiseClimber() {
        
    }
}
