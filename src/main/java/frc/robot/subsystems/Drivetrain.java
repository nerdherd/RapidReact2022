package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Rumble;
import frc.robot.commands.TankDrive;

public class Drivetrain extends SubsystemBase {
  public TalonFX rightMaster;
  public TalonFX leftMaster; 
  public TalonFX rightSlave;
  public TalonFX leftSlave; 

  public Compressor compressor; // Channel 3 on CAN
  public DoubleSolenoid driveShifter; // Channels 0 and 6
  private boolean shifted = false;

  // https://www.kauailabs.com/dist/frc/2022/navx_frc.json
  public DifferentialDriveKinematics m_kinematics; // necessary for autos
  public DifferentialDriveOdometry m_odometry;
  public AHRS m_navx;

  public Drivetrain() {
    rightMaster = new TalonFX(DriveConstants.kRightMasterTalonID);
    leftMaster = new TalonFX(DriveConstants.kLeftMasterTalonID);
    rightSlave = new TalonFX(DriveConstants.kRightSlaveTalonID);  
    leftSlave = new TalonFX(DriveConstants.kLeftSlaveTalonID);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    // Inverted the right side
    leftMaster.setInverted(true);
    rightMaster.setInverted(false);
    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);

    // Pneumatics setup
    compressor = new Compressor(3, PneumaticsModuleType.CTREPCM);
    driveShifter = new DoubleSolenoid(3, PneumaticsModuleType.CTREPCM, 
      DriveConstants.kDriveShifterForwardID, DriveConstants.kDriveShifterReverseID);
    
    m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);
    m_navx = new AHRS(SerialPort.Port.kMXP);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getRawYaw()));
  
  }

  // TODO: Unused method?
  public static double gainInput(double input) {
    input = Math.pow(input, 3);
    return input;
  }

  // =========================== DRIVE =========================== //

  public void setPowerZero() {
    rightMaster.set(ControlMode.PercentOutput, 0);
    leftMaster.set(ControlMode.PercentOutput, 0);
  }

  public void setPower(double rightPower, double leftPower) {
    rightMaster.set(ControlMode.PercentOutput, rightPower);
    leftMaster.set(ControlMode.PercentOutput, leftPower);
  }

  public void tankDrive(double leftInput, double rightInput) {
    double prevLeftOutput = leftMaster.getMotorOutputPercent();
    double prevRightOutput = rightMaster.getMotorOutputPercent();

    // Low pass filter, output = (alpha * intended value) + (1-alpha) * previous value
    double leftOutput = (DriveConstants.kDriveAlpha * leftInput) 
                      + (DriveConstants.kDriveOneMinusAlpha * prevLeftOutput);
    double rightOutput = (DriveConstants.kDriveAlpha * rightInput) 
                      + (DriveConstants.kDriveOneMinusAlpha * prevRightOutput);

    rightMaster.set(ControlMode.PercentOutput, rightOutput);
    leftMaster.set(ControlMode.PercentOutput, leftOutput);
  }

  public void startTankDrive(DoubleSupplier left, DoubleSupplier right) {
    TankDrive tankDrive = new TankDrive(this, left, right);
    tankDrive.schedule();
  }

  public void startRumble(PS4Controller controller) {
    Rumble rumbleCommand = new Rumble(
      () -> rightMaster.getSupplyCurrent(), 
      () -> rightMaster.getSelectedSensorVelocity(), 
      controller, 0);
    
    SequentialCommandGroup initRumble = new SequentialCommandGroup(
      new WaitCommand(1),
      new InstantCommand(() -> rumbleCommand.schedule())
    );

    initRumble.schedule();
  }

  // ========================= SHIFTING ========================= //

  public void shiftHigh() {
    driveShifter.set(Value.kForward);
    shifted = true;
  }

  public void shiftLow() {
    driveShifter.set(Value.kReverse);
    shifted = false;
  }

  public void toggleShift() {
    shifted = !shifted;
    if (shifted) {
      shiftHigh();
    } else {
      shiftLow();
    }
  }

  // =========================== AUTOS =========================== //

  public double getRawYaw() {
    return -m_navx.getAngle();
  }

  public Pose2d getPose2d(){
    return m_odometry.getPoseMeters();
  }

  public double getCurrentX(Pose2d currentPos) {
    return currentPos.getX();
  }

  public double getCurrentY(Pose2d currentPos) {
    return currentPos.getY();
  }

  public double getVelocityFeet(TalonFX talon) {
    return talon.getSelectedSensorVelocity() * 10 / DriveConstants.kTicksPerFoot;
  }

  public DifferentialDriveWheelSpeeds getCurrentSpeeds() {
    return new DifferentialDriveWheelSpeeds(Units.feetToMeters(getVelocityFeet(leftMaster)), Units.feetToMeters(getVelocityFeet(rightMaster)));
  }

  public void setVoltage(double leftVolts, double rightVolts) {
    leftMaster.set(ControlMode.PercentOutput, leftVolts / 12);
    rightMaster.set(ControlMode.PercentOutput, rightVolts / 12);
  }

  // ========================== UTILITY ========================== //

  public void reportToSmartDashboard() {
    SmartDashboard.putNumber(" Right Master Current ", rightMaster.getSupplyCurrent());
    SmartDashboard.putNumber(" Right Slave Current ", rightSlave.getSupplyCurrent());
    SmartDashboard.putNumber(" Left Master Current ", leftMaster.getSupplyCurrent());
    SmartDashboard.putNumber(" Left Slave Current ", leftSlave.getSupplyCurrent());
    SmartDashboard.putNumber(" Drive Velocity ", rightMaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber(" Drive Current ", rightMaster.getSupplyCurrent());
  }

  public void setNeutralCoast() {
    rightMaster.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);
    leftMaster.setNeutralMode(NeutralMode.Coast);
    leftSlave.setNeutralMode(NeutralMode.Coast);
  }  
}
  