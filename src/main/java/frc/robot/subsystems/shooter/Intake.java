package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants; 

public class Intake{

    public TalonSRX intake;
    public boolean intakeIsUp = false;

    public Intake(){
        intake = new TalonSRX(IntakeConstants.kIntakeID);
    }

    public void RaiseIntake(){
        intakeIsUp = true;
        intake.set(ControlMode.MotionMagic, IntakeConstants.kIntakeUpPosition, DemandType.ArbitraryFeedForward, FF());
    }
    public void LowerIntake(){
        intakeIsUp = false;
        intake.set(ControlMode.MotionMagic, IntakeConstants.kIntakeDownPosition, DemandType.ArbitraryFeedForward, FF());
    }

    public void toggleIntake() {
        if (intakeIsUp) {
            LowerIntake();
        } else {
            RaiseIntake();  
        }
    }

    public double ticksToAngle() {
        return 90 - ((intake.getSelectedSensorPosition() - IntakeConstants.kIntakeOffset)
             * 360 / 4096);
    }

    public double degreesToRadians(double deg) {
        return deg * Math.PI/180;
    }

    public double FF() {
        return IntakeConstants.kIntakeGravityFF * Math.cos(degreesToRadians(ticksToAngle()));
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber("FF", FF());
    }
}