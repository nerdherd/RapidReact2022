package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class EverybotIntake extends SubsystemBase {
    public TalonSRX everybotIntake = new TalonSRX(DriveConstants.kEverybotIntake);

    public EverybotIntake() {
        everybotIntake.setInverted(true);
    }

    public double ticksToAngle(double ticksMotor, double ticksAngle) {
        double angle = ticksAngle * ticksMotor / 360 * (Math.PI / 180);
        return angle;
    }

    public void intakeIn(double power) {
        everybotIntake.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber(" Voltage ", power);
    }

    public void intakeOut(double power) {
        everybotIntake.set(ControlMode.PercentOutput, -power);
        SmartDashboard.putNumber(" Voltage ", power);
    }

    public void setPowerZero() {
        everybotIntake.set(ControlMode.PercentOutput, 0);
    }

    // public static void reportToSmartDashboard() {
    //     SmartDashboard.putData(" Time ", );
    // }
}