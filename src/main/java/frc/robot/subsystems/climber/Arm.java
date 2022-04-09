package frc.robot.subsystems.climber;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Log;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;

public class Arm extends SubsystemBase {

    private TalonSRX m_arm;
    
    private DoubleSolenoid climberPiston; // Channels 7 and 6
    private DoubleSolenoid hookPiston; // Channels 2 and 5

    private boolean m_climberShifted = true;
    
    public Arm() {
        m_arm = new TalonSRX(ClimberConstants.kArmTalonID);
        m_arm.setInverted(true);
        
        m_arm.configMotionAcceleration(ClimberConstants.kArmMotionAcceleration);
        m_arm.configMotionCruiseVelocity(ClimberConstants.kArmCruiseVelocity);
        m_arm.configNeutralDeadband(ClimberConstants.kArmDeadband);
        m_arm.config_kP(0, ClimberConstants.kArmkP);
        m_arm.config_kD(0, ClimberConstants.kArmkD);
        m_arm.config_kF(0, ClimberConstants.kArmkF);

        climberPiston = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, DriveConstants.kClimberPistonForwardID, DriveConstants.kClimberPistonReverseID);
        hookPiston = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, DriveConstants.kHookPistonForwardID, DriveConstants.kHookPistonReverseID);

        climberPiston.set(Value.kForward);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new InstantCommand(() ->
        armDefault()));
    }

    public void armDefault() {
        m_arm.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, FF());
    }

    public void setPositionMotionMagic(double ticks) {
        m_arm.set(ControlMode.MotionMagic, ticks, 
            DemandType.ArbitraryFeedForward, FF());
    }

    public double FF() {
        return ClimberConstants.kArmGravityFF * Math.cos(degreesToRadians(ticksToAngle()));
    }

    public double getPosition() {
        return m_arm.getSelectedSensorPosition();
    }
    
    public double ticksToAngle() {
        return 90 - ((m_arm.getSelectedSensorPosition() - ClimberConstants.kArmAngleOffset)
             * 360 / 4096);
    }

    public double degreesToRadians(double deg) {
        return deg * Math.PI/180;
    }  

    public void joystickArmMovement(double armInput) {
        m_arm.set(ControlMode.PercentOutput, armInput * 0.25, DemandType.ArbitraryFeedForward, -1 * FF());
    }

    /** Toggle the climber m_arm hook */
    public void toggleClimberArmHook() {
        m_climberShifted = !m_climberShifted;
        climberPiston.set(m_climberShifted ? Value.kReverse : Value.kForward);
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

    public void resetArmEncoder() {
        m_arm.setSelectedSensorPosition(0);
    }  

    public void setNeutralModeBrake() {
        m_arm.setNeutralMode(NeutralMode.Brake);
    }

    public void setNeutralModeCoast() {
        m_arm.setNeutralMode(NeutralMode.Coast);
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber(" Arm Position ", m_arm.getSelectedSensorPosition());
        SmartDashboard.putNumber(" Arm Velocity ", m_arm.getSelectedSensorVelocity());
        SmartDashboard.putNumber(" Arm Voltage ", m_arm.getMotorOutputVoltage());
        SmartDashboard.putNumber(" Arm Current ", m_arm.getSupplyCurrent());
        SmartDashboard.putNumber(" Climber Angle Conversion ", ticksToAngle());
    }

    public void log(){
        Log.createTopic("Arm Position" + "/Position", () -> m_arm.getSelectedSensorPosition());
        Log.createTopic("Arm Voltage" + "/Voltage", () -> m_arm.getMotorOutputVoltage());
    }

}
