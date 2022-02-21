package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// talon arm w falcon encoder
// falcon elevator w no encoder

public class Elevator {
    
    public static TalonFX elevator = new TalonFX(40);

    /*public void Elevator() {
        elevator = new TalonFX(0);
    }*/

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber(" Elevator Current ", elevator.getSupplyCurrent());
        SmartDashboard.putNumber(" Elevator Voltage ", elevator.getMotorOutputVoltage());
        SmartDashboard.putNumber(" Elevator Position ", elevator.getSelectedSensorPosition());
    }

    public void moveMotionMagic(double position) {
        elevator.set(ControlMode.MotionMagic, position);
    }

    public void moveToPosition(double position) {
        elevator.set(ControlMode.Position, position);
    }

    public static double getPosition() {
        return elevator.getSelectedSensorPosition();
    }

}
