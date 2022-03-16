package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

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

        climberMaster.configMotionCruiseVelocity(400);
        climberMaster.configMotionAcceleration(200);

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

    public void climberUpMotionMagic() {
        double home = climberMaster.getSelectedSensorPosition();

        climberMaster.configMotionCruiseVelocity(4000);
        climberMaster.configMotionAcceleration(2000);
        climberMaster.set(ControlMode.MotionMagic, (home + EverybotConstants.kTicksToLowRung));
        climberMaster.set(ControlMode.MotionMagic, (home + EverybotConstants.kTicksToLowRung));
        climberMaster.set(ControlMode.MotionMagic, (home + EverybotConstants.kTicksToLowRung));
    }

    public void moveClimber(double ticksToTarget) {
        climberMaster.set(ControlMode.MotionMagic, climberMaster.getSelectedSensorPosition() + ticksToTarget);
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
