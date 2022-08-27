package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Log;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  private TalonFX[] m_driveLeftMotors;
  private TalonFX[] m_driveRightMotors;

  private Compressor compressor; // Channel 3 on CAN
  public DoubleSolenoid driveShifter; // Channels 0 and 6

  public Drivetrain() {
    m_driveRightMotors = new TalonFX[] {
      new TalonFX(DriveConstants.kRightMasterTalonID), // Channel 30 on CAN, 14 on PDP
      // new TalonFX(DriveConstants.kRightSlaveTTalonID), // Channel 31 on CAN, 15 on PDP
      new TalonFX(DriveConstants.kRightSlaveBTalonID),
    };

    m_driveLeftMotors = new TalonFX[] {
      new TalonFX(DriveConstants.kLeftMasterTalonID), // Channel 16 on CAN, 0 on PDP
      // new TalonFX(DriveConstants.kLeftSlaveTTalonID), // Channel 17 on CAN, 1 on PDP
      new TalonFX(DriveConstants.kLeftSlaveBTalonID)
    };
    
    m_driveLeftMotors[1].follow(m_driveLeftMotors[0]);
    // m_driveLeftMotors[2].follow(m_driveLeftMotors[0]);
    m_driveRightMotors[1].follow(m_driveRightMotors[0]);
    // m_driveRightMotors[2].follow(m_driveRightMotors[0]);
    
    // Inverted the right side
    m_driveRightMotors[0].setInverted(true);
    for (int i = 1; i <= m_driveRightMotors.length - 1; i++) {
      m_driveRightMotors[i].setInverted(InvertType.FollowMaster);;
    }

    // Pneumatics setup
    compressor = new Compressor(3, PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();

    driveShifter = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, DriveConstants.kDriveShifterForwardID, DriveConstants.kDriveShifterReverseID);
    // MIGHT NEED TO BE CHANGED
    
  }

  // ======================= TELEOP FUNCTIONS ======================= //

  public void setDriveShifterForward() {
    driveShifter.set(Value.kForward);
  }

  public void setDriveShifterReverse() {
    driveShifter.set(Value.kReverse);
  }
  
  
  public void setPower(double leftSpeed, double rightSpeed) {
    m_driveLeftMotors[0].set(ControlMode.PercentOutput, leftSpeed);
    m_driveRightMotors[0].set(ControlMode.PercentOutput, rightSpeed);
  }

  public void setNeutralModes() {
    for (int i = 0; i > m_driveLeftMotors.length; i++) {
      m_driveLeftMotors[i].setNeutralMode(NeutralMode.Coast);
      m_driveRightMotors[i].setNeutralMode(NeutralMode.Coast);
    }
  }

  public double getLeftMotorOutputPercent() {
    return m_driveLeftMotors[0].getMotorOutputPercent();
  }

  public double getRightMotorOutputPercent() {
    return m_driveRightMotors[0].getMotorOutputPercent();
  }

  public void reportToSmartDashboard() {
    SmartDashboard.putNumber(" Right Master Current ",  m_driveRightMotors[0].getSupplyCurrent());
    SmartDashboard.putNumber(" Left Master Current ",   m_driveLeftMotors[1].getSupplyCurrent());
    SmartDashboard.putNumber(" Right Slave T Current ", m_driveRightMotors[1].getSupplyCurrent());
    SmartDashboard.putNumber(" Left Slave T Current ",  m_driveLeftMotors[1].getSupplyCurrent());
    SmartDashboard.putNumber(" Right Slave B Current ", m_driveRightMotors[2].getSupplyCurrent());
    SmartDashboard.putNumber(" Left Slave B Current ",  m_driveLeftMotors[2].getSupplyCurrent());
  }

  public void log() {
    Log.createTopic("RightMaster" + "/Voltage", () -> m_driveRightMotors[0].getMotorOutputVoltage());
    Log.createTopic("LeftMaster" + "/Voltage", () -> m_driveLeftMotors[0].getMotorOutputVoltage());
    Log.createTopic("RightTFollower" + "/Voltage", () -> m_driveRightMotors[1].getMotorOutputVoltage());
    Log.createTopic("LeftTFollower" + "/Voltage", () -> m_driveLeftMotors[1].getMotorOutputVoltage());
    Log.createTopic("RightBFollower" + "/Voltage", () -> m_driveRightMotors[2].getMotorOutputVoltage());
    Log.createTopic("LeftBFollower" + "/Voltage", () -> m_driveLeftMotors[2].getMotorOutputVoltage());
  }

}
