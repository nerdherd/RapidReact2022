package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Log;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.StopElevator;

public class Elevator extends SubsystemBase {
    private TalonSRX m_elevator;

    public Elevator() {
        m_elevator = new TalonSRX(ClimberConstants.kElevatorTalonID);
        m_elevator.setInverted(false);

        m_elevator.configMotionAcceleration(ClimberConstants.kElevatorMotionAcceleration);
        m_elevator.configMotionCruiseVelocity(ClimberConstants.kElevatorCruiseVelocity);
        m_elevator.configNeutralDeadband(ClimberConstants.kElevatorDeadband);
        m_elevator.config_kP(0, ClimberConstants.kElevatorkP);
        m_elevator.config_kD(0, ClimberConstants.kElevatorkD);
        stopElevator();
    }

    public void initDefaultCommand() {
        // setDefaultCommand(new StopElevator(this));
    }
    
    public void stopElevator() {
        m_elevator.set(ControlMode.PercentOutput, 0);
    }
    
    public void moveElevator(double speed) {
        m_elevator.set(ControlMode.PercentOutput, speed);
    }

    public void moveElevatorUp() {
        if (m_elevator.getSelectedSensorPosition() < ClimberConstants.kElevatorTicksUp) {
            m_elevator.set(ControlMode.PercentOutput, 0.32);
            SmartDashboard.putString(" Running Command ", "Elevator Up ");
        } else if (m_elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksUp) {
            m_elevator.set(ControlMode.PercentOutput, 0);
        }
    }

    public void moveElevatorExtend() {
        if (m_elevator.getSelectedSensorPosition() < ClimberConstants.kElevatorTicksExtend) {
            m_elevator.set(ControlMode.PercentOutput, 0.32);
            SmartDashboard.putString(" Running Command ", "Elevator Up Extend ");
        } else if (m_elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksExtend) {
            m_elevator.set(ControlMode.PercentOutput, 0);
        }
    }

    public void moveElevatorDown() {
        if (m_elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksDown ){
            m_elevator.set(ControlMode.PercentOutput, -0.4);
            SmartDashboard.putString(" Running Command ", "Elevator Down ");
        } else if (m_elevator.getSelectedSensorPosition() <= ClimberConstants.kElevatorTicksDown) {
            m_elevator.set(ControlMode.PercentOutput, 0);
        }
    }

    public void setPositionMotionMagic(double ticks){
        m_elevator.set(ControlMode.MotionMagic, ticks);
    }

    public void resetElevatorEncoder() {
        m_elevator.setSelectedSensorPosition(0);
    }

    public void setNeutralModeBrake() {
        m_elevator.setNeutralMode(NeutralMode.Brake);
    }
    
    public void setNeutralModeCoast() {
        m_elevator.setNeutralMode(NeutralMode.Coast);
    }

    public void reportToSmartDashboard(){
        SmartDashboard.putNumber("Elevator Position", m_elevator.getSelectedSensorPosition());
        SmartDashboard.putNumber("Elevator Velocity", m_elevator.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Elevator Current", m_elevator.getSupplyCurrent());
        SmartDashboard.putNumber("Elevator Voltage", m_elevator.getBusVoltage());
    }

    public void log() {
        Log.createTopic("Elevator Position" + ("/Position"), () -> m_elevator.getSelectedSensorPosition());
        Log.createTopic("Elevator Velocity" + ("/Velocity"), () -> m_elevator.getSelectedSensorVelocity());
    }
    
}