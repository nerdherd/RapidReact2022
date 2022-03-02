package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;


public class Arm {
    public static TalonSRX arm = new TalonSRX(15);

    private static double armKp = 1.0;
    private static double armKd = 0.0002;
    private static double armFF = 0;


    public static void armFeedFF() {
        double ks = 0;
        double kcos = 0;
        double kv = 0;
        double armPosRadians = 0;
        double armVel = 0;

        ArmFeedforward ff = new ArmFeedforward(ks, kcos, kv);

        double armCalcFF = ff.calculate(armPosRadians, armVel);

    }
    public static void moveArm(double speed, double waitTime) {
        arm.set(ControlMode.PercentOutput, speed);

        Timer.delay(waitTime);

        arm.set(ControlMode.PercentOutput, 0);
    }

    public static double ticksToAngle(double ticks, double ticksAngle) {
        double angle = ticksAngle * ticks / 360 * (Math.PI / 180);
        return angle;
    }

    public static void rotateArmToAngle(double target, double errorThreshold) {
        double error = Math.abs(target - arm.getSelectedSensorPosition());
        double oldError = Math.abs(target - arm.getSelectedSensorPosition());
        double armSpeed = 0;

        while (error > errorThreshold) {
            oldError = error;
            error = Math.abs(target - arm.getSelectedSensorPosition());
            armSpeed = (armKp * error * 0.001) + (armKd * (oldError - error) * 0.001 / 0.01) + (armFF * Math.cos(ticksToAngle(4096, 90.8) * 0.001));

            if (arm.getSelectedSensorPosition() < target) {
                moveArm(armSpeed, 0.01);
            } else if (arm.getSelectedSensorPosition() > target) {
                moveArm((armSpeed * -1), 0.01);
            }
            
        }

        // Updates SmartDashboard
        SmartDashboard.putNumber(" Target ", target);
        SmartDashboard.putNumber(" Error ", error);
        SmartDashboard.putNumber(" Arm Speed ", armSpeed);
        SmartDashboard.putNumber(" Old Error ", oldError);
        
        if (error < errorThreshold) {
            SmartDashboard.putString(" Pos ", " True ");
        } else {
            SmartDashboard.putString(" Pos ", " False ");
        }
    }
}
