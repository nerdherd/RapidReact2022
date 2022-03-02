package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    public static TalonSRX elevator = new TalonSRX(8);

    private static double elevatorKp = 10;
    private static double elevatorKd = 0.0002;
    private static double elevatorFF = 0;
    
    public static void moveElevator(double speed, double waitTime) {
        elevator.set(ControlMode.PercentOutput, speed);

        Timer.delay(waitTime);

        elevator.set(ControlMode.PercentOutput, 0);
    }

    public static void moveElevatortoPos(double target, double errorThreshold) {
        double error = Math.abs(target - elevator.getSelectedSensorPosition());
        double oldError = Math.abs(target - elevator.getSelectedSensorPosition());
        double elevatorSpeed = 0;

        while (error > errorThreshold) {
            oldError = error;
            error = Math.abs(target - elevator.getSelectedSensorPosition());
            elevatorSpeed = (elevatorKp * error * 0.001) + (elevatorKd * (oldError - error) * 0.001 / 0.01) + (elevatorFF * 0.001);
        }

        SmartDashboard.putNumber(" Elevator Target ", target);
        SmartDashboard.putNumber(" Elevator Error ", error);
        SmartDashboard.putNumber(" Elevator Speed ", elevatorSpeed);
        SmartDashboard.putNumber(" Old Elevator Error ", oldError);

        if (error < errorThreshold) {
            SmartDashboard.putString(" Pos ", " True ");
        } else {
            SmartDashboard.putString(" Pos ", " False ");
        }
    }   
}
