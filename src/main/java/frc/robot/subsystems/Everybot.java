package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.OI;
import frc.robot.Constants.EverybotConstants;

// TODO: Confirm if this file is needed, remove if not. 

public class Everybot extends SubsystemBase {
    public EverybotIntake intake;
    public EverybotArm arm;
    public EverybotClimber climber;
    // public static EverybotArm2 arm2;

    public Everybot() {
        intake = new EverybotIntake();
        arm = new EverybotArm();
        climber = new EverybotClimber();
    }

    public void shooterControllerMovement() {

        if (OI.ps4Controller2.getL1ButtonPressed()) {
            intake.intakeIn(EverybotConstants.kEverybotIntake);
            SmartDashboard.putString(" Button State ", "L1");

        }
        
        if (OI.ps4Controller2.getR1ButtonPressed()) {
            intake.intakeOut(EverybotConstants.kEverybotOuttake);
            SmartDashboard.putString(" Button State ", "L2");

        }

        if (OI.ps4Controller2.getTriangleButtonPressed()) {
            intake.intakeIn(0);
            SmartDashboard.putString(" Button State ", "Triangle");
        }

        if (OI.ps4Controller2.getSquareButtonPressed()) {
            // arm.rotateArmToAngle(EverybotConstants.kHighAngle, EverybotConstants.kHighAngleThreshold);
            // arm.arm.set(ControlMode.PercentOutput, -0.16);
            // EverybotArm2.Arm2Up();
            SmartDashboard.putString(" Button State ", "Square");
        }

        if (OI.ps4Controller2.getCircleButtonPressed()) {
            // arm.rotateArmToAngle(EverybotConstants.kLowAngle, EverybotConstants.kLowAngleThreshold);
            // arm.arm.set(ControlMode.PercentOutput, 0.16);
            // EverybotArm2.Arm2Down();
            SmartDashboard.putString(" Button State " , "Circle");
        }

    }

    public void updateSmartDashboardForEverybot() {
        SmartDashboard.putNumber(" Arm Position ", arm.arm.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Climber Position", climber.climberMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Intake Stator Current ", intake.everybotIntake.getStatorCurrent());
        SmartDashboard.putNumber(" Intake Supply Current ", intake.everybotIntake.getSupplyCurrent());
    }
}
