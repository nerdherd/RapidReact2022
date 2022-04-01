package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Arm;

public class ToggleHookPiston extends CommandBase {
    private final double deadband;
    private final PS4Controller ps4Controller;
    private final Arm arm;

    public ToggleHookPiston(double deadband, PS4Controller ps4Controller, Arm arm) {
        this.deadband = deadband;
        this.ps4Controller = ps4Controller;
        this.arm = arm;
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (ps4Controller.getLeftY() > deadband) {
            arm.hookPiston.set(Value.kForward);
        } else if (ps4Controller.getLeftY() < deadband) {
            arm.hookPiston.set(Value.kReverse);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
