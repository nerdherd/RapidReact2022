package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  public TalonFX[] driveLeftMotors;
  public TalonFX[] driveRightMotors;

  private Compressor compressor; // Channel 3 on CAN
  public DoubleSolenoid driveShifter; // Channels 0 and 6

  public Drivetrain() {
    driveRightMotors = new TalonFX[] {
      new TalonFX(DriveConstants.kRightMasterTalonID), // Channel 30 on CAN, 14 on PDP
      new TalonFX(DriveConstants.kRightSlaveTTalonID), // Channel 31 on CAN, 15 on PDP
      new TalonFX(DriveConstants.kRightSlaveBTalonID),
    };

    driveLeftMotors = new TalonFX[] {
      new TalonFX(DriveConstants.kLeftMasterTalonID), // Channel 16 on CAN, 0 on PDP
      new TalonFX(DriveConstants.kLeftSlaveTTalonID), // Channel 17 on CAN, 1 on PDP
      new TalonFX(DriveConstants.kLeftSlaveBTalonID)
    };
    
    driveLeftMotors[1].follow(driveLeftMotors[0]);
    driveLeftMotors[2].follow(driveLeftMotors[0]);
    driveRightMotors[1].follow(driveRightMotors[0]);
    driveRightMotors[2].follow(driveRightMotors[0]);
    
    // Inverted the right side
    driveRightMotors[0].setInverted(true);
    for (int i = 1; i <= 2; i++) {
      driveRightMotors[i].setInverted(InvertType.FollowMaster);;
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
    driveLeftMotors[0].set(ControlMode.PercentOutput, leftSpeed);
    driveRightMotors[0].set(ControlMode.PercentOutput, rightSpeed);
  }

  public void reportToSmartDashboard() {
    SmartDashboard.putNumber(" Right Master Current ",  driveRightMotors[0].getSupplyCurrent());
    SmartDashboard.putNumber(" Left Master Current ",   driveLeftMotors[1].getSupplyCurrent());
    SmartDashboard.putNumber(" Right Slave T Current ", driveRightMotors[1].getSupplyCurrent());
    SmartDashboard.putNumber(" Left Slave T Current ",  driveLeftMotors[1].getSupplyCurrent());
    SmartDashboard.putNumber(" Right Slave B Current ", driveRightMotors[2].getSupplyCurrent());
    SmartDashboard.putNumber(" Left Slave B Current ",  driveLeftMotors[2].getSupplyCurrent());
  }

}
