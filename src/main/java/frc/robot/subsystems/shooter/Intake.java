package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants; 

public class Intake{

    public TalonSRX intake;
    public boolean intakeIsUp;
    public int intakeTargetPosition;

    public Intake(){
        intake = new TalonSRX(IntakeConstants.kIntakeID);
        intake.setInverted(true);
        intake.configMotionAcceleration(IntakeConstants.kIntakeMotionAcceleration);
        intake.configMotionCruiseVelocity(IntakeConstants.kIntakeCruiseVelocity);
        intake.configNeutralDeadband(IntakeConstants.kIntakeDeadband);
        intake.config_kP(0, IntakeConstants.kIntakeP);
        intake.config_kD(0, IntakeConstants.kIntakeD);
        intake.config_kF(0, IntakeConstants.kIntakeF);
    }

    public void RaiseIntake(){
        intakeIsUp = true;
        intake.set(ControlMode.MotionMagic, (int)IntakeConstants.kIntakeUpPosition, DemandType.ArbitraryFeedForward, FF());
        intakeTargetPosition = (int)IntakeConstants.kIntakeUpPosition;
    }
    public void LowerIntake(){
        intakeIsUp = false;
        intake.set(ControlMode.MotionMagic, IntakeConstants.kIntakeOffset, DemandType.ArbitraryFeedForward, FF());
        intakeTargetPosition = (int)IntakeConstants.kIntakeOffset;
    }
    public void DisableIntake(){
        intake.set(ControlMode.PercentOutput,0);
    }

    public void toggleIntake() {
        if (intakeIsUp) {
            LowerIntake();
        } else {
            RaiseIntake();  
        }
    }

    public double ticksToAngle() {
        double tickDifference = Math.abs(intake.getSelectedSensorPosition() - IntakeConstants.kIntakeOffset);
        double angleDifference = tickDifference * 360 / 4096;
        return 90 - angleDifference;
    }

    public double degreesToRadians(double deg) {
        return deg * Math.PI/180;
    }

    public double FF() {
        return IntakeConstants.kIntakeGravityFF * Math.cos(degreesToRadians(ticksToAngle()));
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber("FF", FF());

        SmartDashboard.putBoolean("Intake reached position", false);

        // Within 10 ticks of target
        if (intake.getSelectedSensorPosition() < intakeTargetPosition + 10 || 
        intake.getSelectedSensorPosition() > intakeTargetPosition - 10) {
            SmartDashboard.putBoolean("Intake reached position", true);
        } else {
            SmartDashboard.putBoolean("Intake reached position", false);
        }
    }

    public void init() {
        // LowerIntake();
        intakeIsUp = false;
        DisableIntake();
    }
    
}