package frc.robot.subsystems.climber.commands.systemchecks;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveMotor extends CommandBase {

    private boolean isCommandFinished = false;

    private final Drivetrain m_driveMotor;

    public DriveMotor (Drivetrain driveMotor) {
        m_driveMotor = driveMotor;
        addRequirements(driveMotor);

    }

    @Override
    public void initialize() {
        
        SmartDashboard.putString(" Command ", " Left Forward ");
        m_driveMotor.drive(0.1, 0, 2);
        SmartDashboard.putString(" Command ", " Right Forward ");
        m_driveMotor.drive(0, 0.1, 2);
        SmartDashboard.putString(" Command ", " Left Back ");
        m_driveMotor.drive(-0.1, 0, 2);
        SmartDashboard.putString(" Command ", " Right Back ");
        m_driveMotor.drive(0, -0.1, 2);
        SmartDashboard.putString(" Command ", " Both Forward ");
        m_driveMotor.drive(0.1, 0.1, 2);
        SmartDashboard.putString(" Command ", " Both Back ");
        m_driveMotor.drive(-0.1, -0.1, 2);

        isCommandFinished = true;
        SmartDashboard.putString(" Command ", " Stopped ");
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
