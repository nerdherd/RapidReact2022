package frc.robot.everybot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.everybot.EverybotHeight;
import frc.robot.everybot.EverybotIntake;

public class Everybot {
    public static EverybotIntake intake;
    public static EverybotHeight arm;

    public static void setUpEverybot() {
        intake = new EverybotIntake();
        arm = new EverybotHeight();
    }

    public static void shooterControllerMovement() {
        double intakePower = 0.5;
        double outtakePower = 0.5;
        double highAngle = 0;
        double lowAngle = 0;

        if (OI.ps4Controller.getL1ButtonPressed()) {
            intake.intakeIn(intakePower);
            SmartDashboard.putString(" Button State ", "L1");
        }
        
        if (OI.ps4Controller.getL2ButtonPressed()) {
            intake.intakeOut(outtakePower);
            SmartDashboard.putString(" Button State ", "L2");
        }

        if (OI.ps4Controller.getR1ButtonPressed()) {
            arm.setPositionMotionMagic(highAngle);
            SmartDashboard.putString(" Button State ", "R1");
            SmartDashboard.putNumber(" High Angle ", highAngle);
        }

        if (OI.ps4Controller.getR2ButtonPressed()) {
            arm.setPositionMotionMagic(lowAngle);
            SmartDashboard.putString(" Button State ", "R2");
            SmartDashboard.putNumber(" Low Angle ", lowAngle);
        }
    }
}
