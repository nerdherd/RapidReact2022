package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  public TalonFX[] driveMotors;

  public Compressor compressor; // Channel 3 on CAN
  public DoubleSolenoid driveShifter; // Channels 0 and 6
  public DoubleSolenoid climberShifter; // Channels 7 and 6
  public DoubleSolenoid hookShifter; // Channels 2 and 5

  public Drivetrain() {
    driveMotors = new TalonFX[] {
      new TalonFX(DriveConstants.kRightMasterTalonID), // Channel 30 on CAN, 14 on PDP
      new TalonFX(DriveConstants.kLeftMasterTalonID), // Channel 16 on CAN, 0 on PDP
      new TalonFX(DriveConstants.kRightSlaveTTalonID), // Channel 31 on CAN, 15 on PDP
      new TalonFX(DriveConstants.kLeftSlaveTTalonID), // Channel 17 on CAN, 1 on PDP
      new TalonFX(DriveConstants.kRightSlaveBTalonID),
      new TalonFX(DriveConstants.kLeftSlaveBTalonID)
    };

    
    driveMotors[3].follow(driveMotors[1]);
    driveMotors[5].follow(driveMotors[1]);
    driveMotors[2].follow(driveMotors[0]);
    driveMotors[4].follow(driveMotors[0]);
    
    // Inverted the right side
    driveMotors[0].setInverted(true);
    for (int i = 3; i <= 6; i++) {
      driveMotors[i].setInverted(InvertType.FollowMaster);;
    }

    // Pneumatics setup
    compressor = new Compressor(3, PneumaticsModuleType.CTREPCM);

    driveShifter = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, DriveConstants.kDriveShifterForwardID, DriveConstants.kDriveShifterReverseID);
    // MIGHT NEED TO BE CHANGED
    climberShifter = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, DriveConstants.kClimberShifterForwardID, DriveConstants.kClimberShifterReverseID);
    hookShifter = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, DriveConstants.kHookShifterForwardID, DriveConstants.kHookShifterReverseID);
  }

  // ======================= TELEOP FUNCTIONS ======================= //

  public void setClimberShifterForward() {
    climberShifter.set(Value.kForward);
  }

  public void setClimberShifterReverse() {
    climberShifter.set(Value.kReverse);
  }

  public void setDriveShifterForward() {
    driveShifter.set(Value.kForward);
  }

  public void setDriveShifterReverse() {
    driveShifter.set(Value.kReverse);
  }
  
  public void setPower(double leftSpeed, double rightSpeed) {
    driveMotors[1].set(ControlMode.PercentOutput, leftSpeed);
    driveMotors[0].set(ControlMode.PercentOutput, rightSpeed);
  }

  public void reportToSmartDashboard() {
    SmartDashboard.putNumber(" Right Master Current ",  driveMotors[0].getSupplyCurrent());
    SmartDashboard.putNumber(" Left Master Current ",   driveMotors[1].getSupplyCurrent());
    SmartDashboard.putNumber(" Right Slave T Current ", driveMotors[2].getSupplyCurrent());
    SmartDashboard.putNumber(" Left Slave T Current ",  driveMotors[3].getSupplyCurrent());
    SmartDashboard.putNumber(" Right Slave B Current ", driveMotors[4].getSupplyCurrent());
    SmartDashboard.putNumber(" Left Slave B Current ",  driveMotors[5].getSupplyCurrent());
  }

}
