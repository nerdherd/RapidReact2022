package frc.robot.subsystems.climber.commands.elevator;

import frc.robot.RobotContainer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;

public class ElevatorExtend extends CommandBase{
    
    private static RobotContainer robotContainer = new RobotContainer();
    
    @Override
    public void execute() {
        if (robotContainer.elevator.elevator.getSelectedSensorPosition() < ClimberConstants.kElevatorTicksExtend) {
            robotContainer.elevator.elevator.set(ControlMode.PercentOutput, 0.4);
            SmartDashboard.putString(" Running Command ", "Elevator Up Extend ");
        } else if (robotContainer.elevator.elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksExtend) {
            SmartDashboard.putString(" Running Command ", "Elevator Up Extend Reached ");
            robotContainer.elevator.elevator.set(ControlMode.PercentOutput, 0);
        }
        SmartDashboard.putString(" Button State ", " Circle ");
    }

}
