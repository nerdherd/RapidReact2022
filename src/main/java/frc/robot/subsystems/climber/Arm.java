package frc.robot.subsystems.climber;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;

public class Arm extends SubsystemBase {

    public TalonSRX arm;
    
    private DoubleSolenoid climberPiston; // Channels 7 and 6
    public DoubleSolenoid hookPiston; // Channels 2 and 5

    private boolean climberShifted = true;
    
    public Arm() {
        arm = new TalonSRX(ClimberConstants.kArmTalonID);
        arm.setInverted(true);
        arm.configMotionAcceleration(ClimberConstants.kArmMotionAcceleration);
        arm.configMotionCruiseVelocity(ClimberConstants.kArmCruiseVelocity);
        arm.configNeutralDeadband(ClimberConstants.kArmDeadband);
        arm.config_kP(0, ClimberConstants.kArmkP);
        arm.config_kD(0, ClimberConstants.kArmkD);
        arm.config_kF(0, ClimberConstants.kArmkF);

        climberPiston = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, DriveConstants.kClimberPistonForwardID, DriveConstants.kClimberPistonReverseID);
        hookPiston = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, DriveConstants.kHookPistonForwardID, DriveConstants.kHookPistonReverseID);

        climberPiston.set(Value.kForward);
    }

    public void setPositionMotionMagic(double ticks) {
        arm.set(ControlMode.MotionMagic, ticks, 
            DemandType.ArbitraryFeedForward, FF());
    }

    public void resetClimbEncoder() {
        arm.setSelectedSensorPosition(0);
    }  

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber(" Climber Position ", arm.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Climber Voltage ", arm.getMotorOutputVoltage());
        SmartDashboard.putNumber(" Climber Voltage ", arm.getSupplyCurrent());
        SmartDashboard.putNumber(" Climber Angle Conversion ", ticksToAngle());
    }

    public double FF() {
        return ClimberConstants.kArmGravityFF * Math.cos(degreesToRadians(ticksToAngle()));
    }

    public double getPosition() {
        return arm.getSelectedSensorPosition();
    }
    
    public double ticksToAngle() {
        return 90 - ((arm.getSelectedSensorPosition() - ClimberConstants.kArmAngleOffset)
             * 360 / 4096);
    }

    public double degreesToRadians(double deg) {
        return deg * Math.PI/180;
    }  

    /** Toggle the climber arm hook */
    public void toggleClimberArmHook() {
        climberShifted = !climberShifted;
        climberPiston.set(climberShifted ? Value.kReverse : Value.kForward);
    }

    public void toggleClimberHook() {
        climberPiston.toggle();
    }

    /** Shift the climber hooks */
    public void enableClimberHooks() {
        hookPiston.set(Value.kForward);
    }

    /** Unshift the climber hooks */
    public void disableClimberHooks() {
        hookPiston.set(Value.kReverse);
    }

}
