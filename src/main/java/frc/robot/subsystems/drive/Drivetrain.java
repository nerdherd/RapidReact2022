package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.OI;

public class Drivetrain {
    public static TalonFX rightMaster; // Channel 30 on CAN, 14 on PDP
    public static TalonFX leftMaster; // Channel 16 on CAN, 0 on PDP
    public static TalonFX rightSlave; // Channel 31 on CAN, 15 on PDP
    public static TalonFX leftSlave; // Channel 17 on CAN, 1 on PDP

    public static Compressor compressor; // Channel 3 on CAN
    private static Piston leftShifter; // Channels 2 and 5
    private static Piston rightShifter; // Channels 1 and 4
    
    // ======================= TELEOP FUNCTIONS ======================= //
    
    public static void setupDrivetrain() {
        rightMaster = new TalonFX(30);
        leftMaster = new TalonFX(16);
        rightSlave = new TalonFX(31);  
        leftSlave = new TalonFX(17);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        // Inverted the right side
        rightMaster.setInverted(true);
        leftSlave.setInverted(InvertType.FollowMaster);
        rightSlave.setInverted(InvertType.FollowMaster);

        // Pneumatics setup
        compressor = new Compressor(3, PneumaticsModuleType.CTREPCM);
        leftShifter = new Piston(3, PneumaticsModuleType.CTREPCM, 2, 5);
        rightShifter = new Piston(3, PneumaticsModuleType.CTREPCM, 1, 4);
    }

    public static void driveControllerMovement() {
        double leftInput = OI.ps4Controller.getLeftY();
        double rightInput = OI.ps4Controller.getRightY();
    
        rightMaster.set(ControlMode.PercentOutput, rightInput);
        leftMaster.set(ControlMode.PercentOutput, leftInput);
    
        // Gear shifting
        // Actually triangle button
        if (OI.ps4Controller.getTriangleButtonPressed()) {
          // Shifts to high gear
          leftShifter.setForwards();
          rightShifter.setForwards();
          SmartDashboard.putString(" Button State ", "A");
          SmartDashboard.putBoolean(" Shifter ", leftShifter.isForwards());
        }

        // Actually square button
        if (OI.ps4Controller.getCircleButtonPressed()) {
          // Shifts to low gear
          leftShifter.setReverse();
          rightShifter.setReverse();
          SmartDashboard.putString(" Button State ", "B");
          SmartDashboard.putBoolean(" Shifter ", leftShifter.isReverse());
        }

    }

    public static void updateSmartDashboardForDrivetrain() {
        SmartDashboard.putNumber(" Right Master Current ", rightMaster.getSupplyCurrent());
        SmartDashboard.putNumber(" Right Slave Current ", rightSlave.getSupplyCurrent());
        SmartDashboard.putNumber(" Left Master Current ", leftMaster.getSupplyCurrent());
        SmartDashboard.putNumber(" Left Slave Current ", leftSlave.getSupplyCurrent());

        SmartDashboard.putBoolean(" Compressor Enabled ", compressor.enabled());
        SmartDashboard.putBoolean(" Pressure Switch ", compressor.getPressureSwitchValue());
        SmartDashboard.putNumber(" Compressor Current ", compressor.getCurrent());
    }

    // ====================== AUTONOMOUS FUNCTIONS ====================== //
    
    // Speed in percentage, waitTime in seconds
    public static void drive(float leftSpeed, float rightSpeed, float waitTime) {
      leftMaster.set(ControlMode.PercentOutput, leftSpeed);
      rightMaster.set(ControlMode.PercentOutput, rightSpeed);

      Timer.delay(waitTime);
    }
}
