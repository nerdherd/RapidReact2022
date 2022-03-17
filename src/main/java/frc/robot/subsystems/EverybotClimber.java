package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EverybotConstants;

public class EverybotClimber extends SubsystemBase {
    
    public TalonFX climberMaster;
    public TalonFX climberSlave;

    public EverybotClimber() {
        climberMaster = new TalonFX(DriveConstants.kClimbMasterTalonID);
        climberSlave = new TalonFX(DriveConstants.kClimbSlaveTalonID);

        // negative goes up, positive goes down when the climber is homed at hardstop
        climberSlave.setInverted(InvertType.OpposeMaster);

        // config tuning params in slot 0
        climberMaster.config_kP(0, EverybotConstants.kEverybotClimberkP);
        climberMaster.config_kI(0, EverybotConstants.kEverybotClimberkI);
        climberMaster.config_kD(0, EverybotConstants.kEverybotClimberkD);
        climberMaster.config_kF(0, EverybotConstants.kEverybotClimberkF);

        climberMaster.configMotionCruiseVelocity(800);
        climberMaster.configMotionAcceleration(400);

        climberSlave.follow(climberMaster);
        /*
        climberSlave.config_kP(0, EverybotConstants.kEverybotClimberkP);
        climberSlave.config_kI(0, EverybotConstants.kEverybotClimberkI);
        climberSlave.config_kD(0, EverybotConstants.kEverybotClimberkD);
        climberSlave.config_kF(0, EverybotConstants.kEverybotClimberkF);

        climberSlave.configMotionCruiseVelocity(100);
        climberSlave.configMotionAcceleration(50);
        */
    }

    public void moveClimber(double ticksToTarget) {
        climberMaster.set(ControlMode.MotionMagic, climberMaster.getSelectedSensorPosition() + ticksToTarget);
    } 
}
