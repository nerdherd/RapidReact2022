package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Log;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EverybotConstants;

public class EverybotClimber extends SubsystemBase {
    
    private TalonFX m_climberMaster;
    private TalonFX m_climberSlave;

    private WPI_TalonFX m_climberMasterSim;
    private WPI_TalonFX m_climberSlaveSim;
    private TalonFXSimCollection m_climberMasterSimData;
    private TalonFXSimCollection m_climberSlaveSimData;

    public EverybotClimber() {
        m_climberMaster = new TalonFX(DriveConstants.kHooksMasterTalonID);
        m_climberSlave = new TalonFX(DriveConstants.kHooksSlaveTalonID);

        // negative goes up, positive goes down when the climber is homed at hardstop
        m_climberSlave.setInverted(InvertType.OpposeMaster);

        // config tuning params in slot 0
        m_climberMaster.config_kP(0, EverybotConstants.kEverybotClimberkP);
        m_climberMaster.config_kI(0, EverybotConstants.kEverybotClimberkI);
        m_climberMaster.config_kD(0, EverybotConstants.kEverybotClimberkD);
        m_climberMaster.config_kF(0, EverybotConstants.kEverybotClimberkF);

        m_climberMaster.configMotionCruiseVelocity(1000); //800
        m_climberMaster.configMotionAcceleration(500); //400

        m_climberSlave.follow(m_climberMaster);

        SmartDashboard.putBoolean("Moving to low rung", false);
        SmartDashboard.putBoolean("Climbing onto low rung", false);

        // Simulation Motor setup

        m_climberMasterSim = new WPI_TalonFX(DriveConstants.kHooksMasterTalonID);
        m_climberSlaveSim = new WPI_TalonFX(DriveConstants.kHooksSlaveTalonID);

        // negative goes up, positive goes down when the climber is homed at hardstop
        m_climberSlaveSim.setInverted(InvertType.OpposeMaster);

        // config tuning params in slot 0
        m_climberMasterSim.config_kP(0, EverybotConstants.kEverybotClimberkP);
        m_climberMasterSim.config_kI(0, EverybotConstants.kEverybotClimberkI);
        m_climberMasterSim.config_kD(0, EverybotConstants.kEverybotClimberkD);
        m_climberMasterSim.config_kF(0, EverybotConstants.kEverybotClimberkF);

        m_climberMasterSim.configMotionCruiseVelocity(1000); //800
        m_climberMasterSim.configMotionAcceleration(500); //400

        m_climberSlaveSim.follow(m_climberMasterSim);
    }

    public void initDefaultCommand() {
        m_climberMaster.set(ControlMode.PercentOutput, 0);
    }

    public void moveClimber(double ticksToTarget) {
        m_climberMaster.set(ControlMode.MotionMagic, m_climberMaster.getSelectedSensorPosition() + ticksToTarget);
    } 

    public void simPeriodic() {
        m_climberMasterSimData.setIntegratedSensorRawPosition((int)(m_climberMasterSim.getSelectedSensorPosition()));
        m_climberMasterSimData.setIntegratedSensorVelocity((int)(m_climberMasterSim.getSelectedSensorVelocity()));
    }

    public void moveClimberSim(double ticksToTarget) {
        m_climberMasterSim.set(ControlMode.Position, m_climberMaster.getSelectedSensorPosition() + ticksToTarget);
    } 

    public void resetEverybotClimberEncoder() {
        m_climberMaster.setSelectedSensorPosition(0);
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber(" Climber Position", m_climberMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Climber Voltage ", m_climberMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber(" Climber Current ", m_climberMaster.getSupplyCurrent());
    }

    public void log() {
        Log.createTopic("ClimberSensorVelocity" + "/Velocity", () -> m_climberMaster.getSelectedSensorVelocity());
        Log.createTopic("ClimberTrajVelocity" + "/Velocity", () -> m_climberMaster.getActiveTrajectoryVelocity());
        Log.createTopic("ClimberSensorPosition" + "/Position", () -> m_climberMaster.getSelectedSensorPosition());
        Log.createTopic("ClimberTrajPosition" + "/Position", () -> m_climberMaster.getActiveTrajectoryPosition());
        Log.createTopic("ClimberOutputVoltage" + "/Voltage", () -> m_climberMaster.getMotorOutputVoltage());

    }
}
