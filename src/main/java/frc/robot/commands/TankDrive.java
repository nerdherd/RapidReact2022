package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
    Drivetrain drivetrain;
    DoubleSupplier leftInput;
    DoubleSupplier rightInput;

    public TankDrive(Drivetrain drivetrain, DoubleSupplier leftInput, DoubleSupplier rightInput) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    public void initialize() {}

    public void execute() {
        drivetrain.tankDrive(leftInput.getAsDouble(), rightInput.getAsDouble());
    }

    public boolean isFinished() {
        return false;
    }
}
