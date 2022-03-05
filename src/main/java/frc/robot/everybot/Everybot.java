package frc.robot.everybot;

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
        double intakePower = 0.5;
        double outtakePower = 0.5;
        double highAngle = 0;
        double lowAngle = 0;

        if (OI.ps4Controller2.getL1ButtonPressed()) {
            EverybotIntake.intakeIn(intakePower);
            SmartDashboard.putString(" Button State ", "L1");
        }
        
        if (OI.ps4Controller2.getL2ButtonPressed()) {
            EverybotIntake.intakeOut(outtakePower);
            SmartDashboard.putString(" Button State ", "L2");
        }

        if (OI.ps4Controller2.getR1ButtonPressed()) {
            arm.rotateArmToAngle(highAngle, 5);
            SmartDashboard.putString(" Button State ", "R1");
            SmartDashboard.putNumber(" High Angle ", highAngle);
        }

        if (OI.ps4Controller2.getR2ButtonPressed()) {
            arm.rotateArmToAngle(lowAngle, 5);
            SmartDashboard.putString(" Button State ", "R2");
            SmartDashboard.putNumber(" Low Angle ", lowAngle);
        }
    }
}
