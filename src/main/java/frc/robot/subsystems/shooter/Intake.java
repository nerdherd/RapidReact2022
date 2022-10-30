package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RollerConstants; 

public class Intake{

    public TalonSRX intake;
    public TalonSRX roller;
    public boolean intakeIsUp;
    public int intakeTargetPosition;

    public Intake(){
        intake = new TalonSRX(IntakeConstants.kIntakeID);
        roller = new TalonSRX(RollerConstants.kRollerID);
        intake.setInverted(true);
        roller.setInverted(true);
        intake.configMotionAcceleration(IntakeConstants.kIntakeMotionAcceleration);
        intake.configMotionCruiseVelocity(IntakeConstants.kIntakeCruiseVelocity);
        intake.configNeutralDeadband(IntakeConstants.kIntakeDeadband);
        intake.config_kP(0, IntakeConstants.kIntakeP);
        intake.config_kD(0, IntakeConstants.kIntakeD);
        intake.config_kF(0, IntakeConstants.kIntakeF);
    }

    public void setIntakePercent(double percent) {
        if (FF() < 0) {
            intake.set(ControlMode.PercentOutput, percent);
        } else {
            intake.set(ControlMode.PercentOutput, percent + FF());
        }
    }

    public void setRollerPercent(double power) {
        roller.set(ControlMode.PercentOutput, power);
    }

    public void setRollerPercentZero() {
        setRollerPercent(0);
    }

    public void ReadyIntake(){
        intakeIsUp = true;
        intake.set(ControlMode.MotionMagic, (int)IntakeConstants.kIntakeReadyPosition);
        intakeTargetPosition = (int)IntakeConstants.kIntakeReadyPosition;
        // setRollerPercent(IntakeConstants.kRollerPercent);
        
    }
    public void StowIntake(){
        intakeIsUp = false;
        intake.set(ControlMode.MotionMagic, IntakeConstants.kIntakeStowPosition);
        intakeTargetPosition = (int)IntakeConstants.kIntakeStowPosition;
        // setRollerPercentZero();
    }

    public void DisableIntake(){
        intake.set(ControlMode.PercentOutput,0);
        setRollerPercentZero();
    }

    public void toggleIntake() {
        if (intakeIsUp) {
            StowIntake();
        } else {
            ReadyIntake();  
        }
    }

    public double ticksToAngle() {
        double tickDifference = Math.abs(intake.getSelectedSensorPosition() - IntakeConstants.kIntakeHorizontal);
        double angleDifference = tickDifference * 360 / 4096;
        return angleDifference;
        // return angleDifference;
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