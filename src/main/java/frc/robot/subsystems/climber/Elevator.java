package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Elevator extends SubsystemBase {
    public TalonSRX elevator = new TalonSRX(ClimberConstants.kElevatorTalonID);

    public Elevator() {
        elevator.setInverted(true);
    }
    
    public void moveElevator(double speed) {
        elevator.set(ControlMode.PercentOutput, speed);
    }

    public void moveElevatorUp() {
        if (elevator.getSelectedSensorPosition() < ClimberConstants.kElevatorTicksUp) {
            elevator.set(ControlMode.PercentOutput, 0.32);
            SmartDashboard.putString(" Running Command ", "Elevator Up ");
        } else if (elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksUp) {
            elevator.set(ControlMode.PercentOutput, 0);
        }
    }

    public void moveElevatorExtend() {
        if (elevator.getSelectedSensorPosition() < ClimberConstants.kElevatorTicksExtend) {
            elevator.set(ControlMode.PercentOutput, 0.32);
            SmartDashboard.putString(" Running Command ", "Elevator Up Extend ");
        } else if (elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksExtend) {
            elevator.set(ControlMode.PercentOutput, 0);
        }
    }

    public void moveElevatorDown() {
        if (elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksDown ){
            elevator.set(ControlMode.PercentOutput, -0.4);
            SmartDashboard.putString(" Running Command ", "Elevator Down ");
        } else if (elevator.getSelectedSensorPosition() <= ClimberConstants.kElevatorTicksDown) {
            elevator.set(ControlMode.PercentOutput, 0);
        }
    }
    
    public void resetElevatorEncoder() {
        elevator.setSelectedSensorPosition(0);
    }
}