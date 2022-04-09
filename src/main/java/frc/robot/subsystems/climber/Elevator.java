package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Elevator extends SubsystemBase {
    public TalonSRX elevator;

    public Elevator() {
        elevator = new TalonSRX(ClimberConstants.kElevatorTalonID);
        elevator.setInverted(false);

        elevator.configMotionAcceleration(ClimberConstants.kElevatorMotionAcceleration);
        elevator.configMotionCruiseVelocity(ClimberConstants.kElevatorCruiseVelocity);
        elevator.configNeutralDeadband(ClimberConstants.kElevatorDeadband);
        elevator.config_kP(0, ClimberConstants.kElevatorkP);
        elevator.config_kD(0, ClimberConstants.kElevatorkD);
    }

    public void initDefaultCommand() { 
        setDefaultCommand(new InstantCommand(() -> 
        moveElevator(0)));
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

    public void setPositionMotionMagic(double ticks){
        elevator.set(ControlMode.MotionMagic, ticks);
    }

    public void resetElevatorEncoder() {
        elevator.setSelectedSensorPosition(0);
    }

    public double testLog() {
        return elevator.getSelectedSensorVelocity();
    }
}