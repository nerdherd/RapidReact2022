package frc.robot.everybot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.OI;
import frc.robot.Constants.EverybotConstants;

public class Everybot extends SubsystemBase {
    public static EverybotIntake intake;
    public static EverybotArm arm;
    public static EverybotClimber climber;
    // public static EverybotArm2 arm2;

    public static void setUpEverybot() {
        intake = new EverybotIntake();
        arm = new EverybotArm();
        climber = new EverybotClimber();
    }

    public void intakeIn() {
        
    }

    public static void shooterControllerMovement() {

        if (OI.ps4Controller2.getL1ButtonPressed()) {
            EverybotIntake.intakeIn(EverybotConstants.kEverybotIntake);
            SmartDashboard.putString(" Button State ", "L1");

        }
        
        if (OI.ps4Controller2.getR1ButtonPressed()) {
            EverybotIntake.intakeOut(EverybotConstants.kEverybotOuttake);
            SmartDashboard.putString(" Button State ", "L2");

        }

        if (OI.ps4Controller2.getL2ButtonPressed()) {
            // EverybotClimber.setPower(EverybotConstants.kEverybotClimberUp);
            EverybotClimber.climberUp();
            
        }

        if (OI.ps4Controller2.getR2ButtonPressed()) {
            // EverybotClimber.setPower(EverybotConstants.kEverybotClimberDown);
            EverybotClimber.climberDown();
        }

        if (OI.ps4Controller2.getTriangleButtonPressed()) {
            EverybotIntake.intakeIn(0);
            SmartDashboard.putString(" Button State ", "Triangle");
        }

        if (OI.ps4Controller2.getSquareButtonPressed()) {
            // EverybotArm.rotateArmToAngle(EverybotConstants.kHighAngle, EverybotConstants.kHighAngleThreshold);
            // EverybotArm.arm.set(ControlMode.PercentOutput, -0.16);
            // EverybotArm2.Arm2Up();
            SmartDashboard.putString(" Button State ", "Square");
        }

        if (OI.ps4Controller2.getCircleButtonPressed()) {
            // EverybotArm.rotateArmToAngle(EverybotConstants.kLowAngle, EverybotConstants.kLowAngleThreshold);
            // EverybotArm.arm.set(ControlMode.PercentOutput, 0.16);
            // EverybotArm2.Arm2Down();
            SmartDashboard.putString(" Button State " , "Circle");
        }

    }

    public static void updateSmartDashboardForEverybot() {
        SmartDashboard.putNumber(" Arm Position ", arm.arm.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Climber Position", climber.climberMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Intake Stator Current ", intake.everybotIntake.getStatorCurrent());
        SmartDashboard.putNumber(" Intake Supply Current ", intake.everybotIntake.getSupplyCurrent());
    }
}
