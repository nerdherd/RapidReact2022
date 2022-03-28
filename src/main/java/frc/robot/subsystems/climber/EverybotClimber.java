package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EverybotConstants;

public class EverybotClimber extends SubsystemBase {
    
    public TalonFX climberMaster;
    public TalonFX climberSlave;

    public EverybotClimber() {
        climberMaster = new TalonFX(DriveConstants.kHooksMasterTalonID);
        climberSlave = new TalonFX(DriveConstants.kHooksSlaveTalonID);

        // negative goes up, positive goes down when the climber is homed at hardstop
        climberSlave.setInverted(InvertType.OpposeMaster);

        // config tuning params in slot 0
        climberMaster.config_kP(0, EverybotConstants.kEverybotClimberkP);
        climberMaster.config_kI(0, EverybotConstants.kEverybotClimberkI);
        climberMaster.config_kD(0, EverybotConstants.kEverybotClimberkD);
        climberMaster.config_kF(0, EverybotConstants.kEverybotClimberkF);

        climberMaster.configMotionCruiseVelocity(1000); //800
        climberMaster.configMotionAcceleration(500); //400

        climberSlave.follow(climberMaster);

        SmartDashboard.putBoolean("Moving to low rung", false);
        SmartDashboard.putBoolean("Climbing onto low rung", false);
    }

    public void moveClimber(double ticksToTarget) {
        climberMaster.set(ControlMode.MotionMagic, climberMaster.getSelectedSensorPosition() + ticksToTarget);
    } 
}
