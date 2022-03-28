package frc.robot.subsystems;

import java.util.Set;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    public TalonFX rightMaster; // Channel 30 on CAN, 14 on PDP
    public TalonFX leftMaster; // Channel 16 on CAN, 0 on PDP
    public TalonFX rightSlaveT; // Channel 31 on CAN, 15 on PDP
    public TalonFX leftSlaveT; // Channel 17 on CAN, 1 on PDP
    public TalonFX rightSlaveB;
    public TalonFX leftSlaveB;

    public Compressor compressor; // Channel 3 on CAN
    public DoubleSolenoid driveShifter; // Channels 0 and 6
    public DoubleSolenoid climberShifter; // Channels 7 and 6
    public DoubleSolenoid hookShifter; // Channels 2 and 5


    // private double modLeftInput;
    // private double modRightInput;
    // private boolean highGear;
    
    // ======================= TELEOP FUNCTIONS ======================= //
    
    public Drivetrain() {
      rightMaster = new TalonFX(DriveConstants.kRightMasterTalonID);
      leftMaster = new TalonFX(DriveConstants.kLeftMasterTalonID);
      rightSlaveT = new TalonFX(DriveConstants.kRightSlaveTTalonID);  
      leftSlaveT = new TalonFX(DriveConstants.kLeftSlaveTTalonID);
      rightSlaveB = new TalonFX(DriveConstants.kRightSlaveBTalonID);
      leftSlaveB = new TalonFX(DriveConstants.kLeftSlaveBTalonID);

      leftSlaveT.follow(leftMaster);
      leftSlaveB.follow(leftMaster);
      rightSlaveT.follow(rightMaster);
      rightSlaveB.follow(rightMaster);

      // Inverted the right side
      rightMaster.setInverted(true);
      leftSlaveT.setInverted(InvertType.FollowMaster);
      leftSlaveB.setInverted(InvertType.FollowMaster);
      rightSlaveT.setInverted(InvertType.FollowMaster);
      rightSlaveB.setInverted(InvertType.FollowMaster);

      // Pneumatics setup
      compressor = new Compressor(3, PneumaticsModuleType.CTREPCM);

      driveShifter = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, DriveConstants.kDriveShifterForwardID, DriveConstants.kDriveShifterReverseID);
      // MIGHT NEED TO BE CHANGED
      climberShifter = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, DriveConstants.kClimberShifterForwardID, DriveConstants.kClimberShifterReverseID);
      hookShifter = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, DriveConstants.kHookShifterForwardID, DriveConstants.kHookShifterReverseID);

    }

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

    public double gainInput(double input) {
      input = Math.pow(input, 3);
      return input;
    }

    public void setPowerZero() {
      rightMaster.set(ControlMode.PercentOutput, 0);
      leftMaster.set(ControlMode.PercentOutput, 0);
    }

    public void setPower(double rightPower, double leftPower) {
      rightMaster.set(ControlMode.PercentOutput, rightPower);
      leftMaster.set(ControlMode.PercentOutput, leftPower);
    }

    public void reportToSmartDashboard() {
      SmartDashboard.putNumber(" Right Master Current ", rightMaster.getSupplyCurrent());
      SmartDashboard.putNumber(" Right Slave Current ", rightSlaveT.getSupplyCurrent());
      SmartDashboard.putNumber(" Right Slave Current ", rightSlaveB.getSupplyCurrent());
      SmartDashboard.putNumber(" Left Master Current ", leftMaster.getSupplyCurrent());
      SmartDashboard.putNumber(" Left Slave Current ", leftSlaveT.getSupplyCurrent());
      SmartDashboard.putNumber(" Left Slave Current ", leftSlaveB.getSupplyCurrent());
      
      // SmartDashboard.putBoolean(" Compressor Enabled ", compressor.enabled());
      // SmartDashboard.putBoolean(" Pressure Switch ", compressor.getPressureSwitchValue());
      // SmartDashboard.putNumber(" Compressor Current ", compressor.getCurrent());

      // SmartDashboard.putNumber( " Left Axis ", robotContainer.OI.ps4Controller.getLeftY());
      // SmartDashboard.putNumber( " Right Axis ", robotContainer.OI.ps4Controller.getRightY());
      // SmartDashboard.putNumber( " Modified Left Axis", leftmod)
    }

    // ====================== AUTONOMOUS FUNCTIONS ====================== //
    
    // Speed in percentage, waitTime in seconds
    public void drive(double leftSpeed, double rightSpeed, double waitTime) {
      leftMaster.set(ControlMode.PercentOutput, leftSpeed);
      rightMaster.set(ControlMode.PercentOutput, rightSpeed);

      Timer.delay(waitTime);
    }
}
