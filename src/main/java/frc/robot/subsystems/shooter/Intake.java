package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants; 

public class Intake{

    public TalonSRX intake;
    public boolean intakeIsUp = false;
    public int intakePosition;

    public Intake(){
        intake = new TalonSRX(IntakeConstants.kIntakeID);
        intake.setInverted(true);
        intake.configMotionAcceleration(IntakeConstants.kIntakeMotionAcceleration);
        intake.configMotionCruiseVelocity(IntakeConstants.kIntakeCruiseVelocity);
        intake.configNeutralDeadband(IntakeConstants.kIntakeDeadband);
        intake.config_kP(0, IntakeConstants.kIntakeP);
        intake.config_kD(0, IntakeConstants.kIntakeD);
        intake.config_kF(0, IntakeConstants.kIntakeF);

        intakePosition = -2564;

    }

    public void RaiseIntake(){
        intakeIsUp = true;
        intake.set(ControlMode.MotionMagic, intakePosition, DemandType.ArbitraryFeedForward, FF());
        intakePosition -= 20;
    }
    public void LowerIntake(){
        intakeIsUp = false;
        intake.set(ControlMode.MotionMagic, intakePosition, DemandType.ArbitraryFeedForward, FF());
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