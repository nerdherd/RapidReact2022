package frc.robot.commands.systemchecks;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EverybotConstants;
import frc.robot.subsystems.EverybotClimber;

public class EverybotMotor extends CommandBase {

    private final EverybotClimber m_everybotMotor;

    public EverybotMotor (EverybotClimber everybotMotor) {
        m_everybotMotor = everybotMotor;
        addRequirements(m_everybotMotor);

    }

    @Override
    public void initialize() {
        
        SmartDashboard.putString(" Command ", " Started ");
        m_everybotMotor.moveClimber(100);
       
        SmartDashboard.putString(" Command ", " Stopped ");
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
