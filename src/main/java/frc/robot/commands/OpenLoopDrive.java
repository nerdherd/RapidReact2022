package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class OpenLoopDrive extends CommandBase {
    private Drivetrain m_drivetrain = new Drivetrain();
    private static double m_power;

    public OpenLoopDrive(double power) {
        m_power = power;
        addRequirements(m_drivetrain);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      m_drivetrain.setPower(m_power, m_power);
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
