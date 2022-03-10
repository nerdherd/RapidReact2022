package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.AbstractDrivetrain;

public class OpenLoopDrive extends CommandBase {
    private AbstractDrivetrain m_drive;
    private static double m_power;

    public OpenLoopDrive(AbstractDrivetrain drive, double power){
        m_power = power;
        m_drive = drive;
        addRequirements(m_drive);
    }

    // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_drive.setPower(m_power, m_power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
  
}
