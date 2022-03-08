package frc.robot.everybot;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.everybot.EverybotHeight;
import frc.robot.everybot.EverybotIntake;

public class Everybot {
    public static EverybotIntake intake;
    public static EverybotArm arm;

    public static void setUpEverybot() {
        intake = new EverybotIntake();
        arm = new EverybotArm();
    }

    public static void shooterControllerMovement() {
        
        final double lowAngle = 0;
        final double highAngle = -20073.00;

        if (OI.ps4Controller2.getL1ButtonPressed()) {
            EverybotIntake.intakeIn(0.6);
            SmartDashboard.putString(" Button State ", "L1");
            SmartDashboard.putNumber(" Stator Current ", intake.everybotIntake.getStatorCurrent());
            SmartDashboard.putNumber(" Supply Current ", intake.everybotIntake.getSupplyCurrent());

        }
        
        if (OI.ps4Controller2.getR1ButtonPressed()) {
            EverybotIntake.intakeOut(0.5);
            SmartDashboard.putString(" Button State ", "L2");
            SmartDashboard.putNumber(" Stator Current ", intake.everybotIntake.getStatorCurrent());
            SmartDashboard.putNumber(" Supply Current ", intake.everybotIntake.getSupplyCurrent());

        }

        if (OI.ps4Controller2.getTriangleButtonPressed()) {
            EverybotIntake.intakeIn(0);
            SmartDashboard.putString(" Button State ", "Triangle");
        }

        if (OI.ps4Controller2.getSquareButtonPressed()) {
            // EverybotArm.rotateArmToAngle(highAngle, 10);
            EverybotArm.arm.set(ControlMode.PercentOutput, -0.16);
            SmartDashboard.putString(" Button State ", "Square");
        }

        if (OI.ps4Controller2.getCircleButtonPressed()) {
            // EverybotArm.rotateArmToAngle(lowAngle, 10);
            EverybotArm.arm.set(ControlMode.PercentOutput, 0.16);
            SmartDashboard.putString(" Button State " , "Circle");
        }

    }

    public static void updateSmartDashboardForEverybot() {
        SmartDashboard.putNumber(" Arm Position ", arm.arm.getSelectedSensorPosition());
    }
}
