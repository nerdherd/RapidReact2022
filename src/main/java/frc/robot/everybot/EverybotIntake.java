package frc.robot.everybot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;

public class EverybotIntake extends SubsystemBase implements Sendable {
    public static TalonSRX everybotIntake = new TalonSRX(RobotMap.kEverybotIntake);

    public EverybotIntake() {
        everybotIntake.setInverted(true);
    }

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

    public static void setPowerZero() {
        everybotIntake.set(ControlMode.PercentOutput, 0);
    }

    // public static void reportToSmartDashboard() {
    //     SmartDashboard.putData(" Time ", );
    // }
}