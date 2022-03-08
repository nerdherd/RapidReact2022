package frc.robot.everybot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class EverybotIntake {
    public static TalonSRX everybotIntake = new TalonSRX(RobotMap.kEverybotIntake);

    public static double ticksToAngle(double ticksMotor, double ticksAngle) {
        double angle = ticksAngle * ticksMotor / 360 * (Math.PI / 180);
        return angle;
    }

    public static void intakeIn(double power) {
        everybotIntake.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber(" Voltage ", power);
    }

    public static void intakeOut(double power) {
        everybotIntake.set(ControlMode.PercentOutput, -power);
        SmartDashboard.putNumber(" Voltage ", power);
    }

}