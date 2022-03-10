package frc.robot.subsystems.drive;

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

import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    public static TalonFX rightMaster; // Channel 30 on CAN, 14 on PDP
    public static TalonFX leftMaster; // Channel 16 on CAN, 0 on PDP
    public static TalonFX rightSlave; // Channel 31 on CAN, 15 on PDP
    public static TalonFX leftSlave; // Channel 17 on CAN, 1 on PDP

    public static Compressor compressor; // Channel 3 on CAN
    private static DoubleSolenoid leftShifter; // Channels 2 and 5
    private static DoubleSolenoid rightShifter; // Channels 1 and 4

    private double modLeftInput;
    private double modRightInput;
    private static boolean highGear;
    
    // ======================= TELEOP FUNCTIONS ======================= //
    
    public static void setupDrivetrain() {
      rightMaster = new TalonFX(RobotMap.kRightMasterTalonID);
      leftMaster = new TalonFX(RobotMap.kLeftMasterTalonID);
      rightSlave = new TalonFX(RobotMap.kRightSlaveTalonID);  
      leftSlave = new TalonFX(RobotMap.kLeftSlaveTalonID);

      leftSlave.follow(leftMaster);
      rightSlave.follow(rightMaster);

      // Inverted the right side
      rightMaster.setInverted(true);
      leftSlave.setInverted(InvertType.FollowMaster);
      rightSlave.setInverted(InvertType.FollowMaster);

      // Pneumatics setup
      compressor = new Compressor(3, PneumaticsModuleType.CTREPCM);
      leftShifter = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, 2, 5);
      rightShifter = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, 1, 4);
    }

    public static double gainInput(double input) {
      input = Math.pow(input, 3);
      return input;
    }

    public static void driveControllerMovement() {
      double leftInput = OI.ps4Controller.getLeftY();
      double rightInput = OI.ps4Controller.getRightY();
      double prevLeftOutput = leftMaster.getMotorOutputPercent();
      double prevRightOutput = rightMaster.getMotorOutputPercent();

      // Low pass filter, output = (alpha * intended value) + (1-alpha) * previous value
      double leftOutput = (DriveConstants.kDriveAlpha * leftInput) + (DriveConstants.kDriveOneMinusAlpha * prevLeftOutput);
      double rightOutput = (DriveConstants.kDriveAlpha * rightInput) + (DriveConstants.kDriveOneMinusAlpha * prevRightOutput);

      // double modLeftInput = gainInput(leftInput);
      // double modRightInput = gainInput(rightInput);

      // double modScale = 0.05;
      
      // if (highGear == true) {
      //   rightMaster.set(ControlMode.PercentOutput, modLeftInput * modScale);
      //   leftMaster.set(ControlMode.PercentOutput, modRightInput * modScale);
      // }

      // if (highGear == false) {
      //   rightMaster.set(ControlMode.PercentOutput, rightInput);
      //   leftMaster.set(ControlMode.PercentOutput, leftInput);
      // }

      rightMaster.set(ControlMode.PercentOutput, rightOutput);
      leftMaster.set(ControlMode.PercentOutput, leftOutput);

  
      // Gear shifting
      // Actually triangle button
      if (OI.ps4Controller.getTriangleButtonPressed()) {
        // Shifts to high gear
        leftShifter.set(Value.kForward);
        rightShifter.set(Value.kForward);
        SmartDashboard.putString(" Button State ", "A");
        // highGear = true;
      }

      // Actually square button
      if (OI.ps4Controller.getCircleButtonPressed()) {
        // Shifts to low gear
        leftShifter.set(Value.kReverse);
        rightShifter.set(Value.kReverse);
        SmartDashboard.putString(" Button State ", "B");
        // highGear = false;
      }
    }

    public static void setPowerZero() {
      rightMaster.set(ControlMode.PercentOutput, 0);
      leftMaster.set(ControlMode.PercentOutput, 0);
    }

    public static void updateSmartDashboardForDrivetrain() {
      SmartDashboard.putNumber(" Right Master Current ", rightMaster.getSupplyCurrent());
      SmartDashboard.putNumber(" Right Slave Current ", rightSlave.getSupplyCurrent());
      SmartDashboard.putNumber(" Left Master Current ", leftMaster.getSupplyCurrent());
      SmartDashboard.putNumber(" Left Slave Current ", leftSlave.getSupplyCurrent());
        
      SmartDashboard.putBoolean(" Compressor Enabled ", compressor.enabled());
      SmartDashboard.putBoolean(" Pressure Switch ", compressor.getPressureSwitchValue());
      SmartDashboard.putNumber(" Compressor Current ", compressor.getCurrent());

      SmartDashboard.putNumber( " Left Axis ", OI.ps4Controller.getLeftY());
      SmartDashboard.putNumber( " Right Axis ", OI.ps4Controller.getRightY());
      // SmartDashboard.putNumber( " Modified Left Axis", leftmod)
    }

    // ====================== AUTONOMOUS FUNCTIONS ====================== //
    
    // Speed in percentage, waitTime in seconds
    public static void drive(double leftSpeed, double rightSpeed, double waitTime) {
      leftMaster.set(ControlMode.PercentOutput, leftSpeed);
      rightMaster.set(ControlMode.PercentOutput, rightSpeed);

      Timer.delay(waitTime);
    }
}
