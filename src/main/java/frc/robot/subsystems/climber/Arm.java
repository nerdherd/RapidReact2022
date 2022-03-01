package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
    public static TalonSRX arm = new TalonSRX(15);

    private static double armKp;
    private static double armKd;

    public static void spinArm(double speed, double waitTime) {
        arm.set(ControlMode.PercentOutput, speed);

        Timer.delay(waitTime);
    }

    public static void rotateArmToAngle(double target, double errorThreshold) {
        double error = 0;
        double oldError = 0;
        double armSpeed = 0;

        while (error > Math.abs(errorThreshold)) {
            oldError = error;
            error = arm.getSelectedSensorPosition();
            armSpeed = (armKp * error) + (armKd * (oldError - error) * 0.001);

            spinArm(armSpeed, 0.001);
        }

        // Updates SmartDashboard
        SmartDashboard.putNumber(" Arm Speed ", armSpeed);
        SmartDashboard.putNumber(" Old Error ", oldError);
        SmartDashboard.putNumber(" Error ", error);
    }
}
