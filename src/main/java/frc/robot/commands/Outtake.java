package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.everybot.EverybotIntake;

public class Outtake extends CommandBase {
    private static double m_power;
    private EverybotIntake m_intake;

    public Outtake(double power) {
        m_power = power;
        // m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    EverybotIntake.intakeOut(m_power);
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
