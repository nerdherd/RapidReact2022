package frc.robot.subsystems.climber.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorDown extends CommandBase {
    
    private static RobotContainer robotContainer = new RobotContainer();

    @Override
    public void execute() {
        if (robotContainer.elevator.elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksDown ){
            robotContainer.elevator.elevator.set(ControlMode.PercentOutput, -0.5);
            SmartDashboard.putString(" Running Command ", "Elevator Down ");
        } else if (robotContainer.elevator.elevator.getSelectedSensorPosition() <= ClimberConstants.kElevatorTicksDown) {
            robotContainer.elevator.elevator.set(ControlMode.PercentOutput, 0);
            SmartDashboard.putString( " Running Command ", " Elevator Reached Down Position ");
        }
        SmartDashboard.putString( "Button State ", "Square ");
    }

}
