package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Log;
import frc.robot.Constants.ClimberConstants;

public class Elevator extends SubsystemBase {
    private TalonSRX m_elevator;

    private WPI_TalonSRX m_elevatorMotorSim;
    private TalonSRXSimCollection m_elevatorSimData;

    private ElevatorSim m_elevatorSim;

    private Mechanism2d m_mech2D;
    private MechanismRoot2d m_mechRoot2D;
    private MechanismLigament2d m_elevatorMech2D;

    public Elevator() {
        m_elevator = new TalonSRX(ClimberConstants.kElevatorTalonID);
        m_elevator.setInverted(false);

        m_elevator.configMotionAcceleration(ClimberConstants.kElevatorMotionAcceleration);
        m_elevator.configMotionCruiseVelocity(ClimberConstants.kElevatorCruiseVelocity);
        m_elevator.configNeutralDeadband(ClimberConstants.kElevatorDeadband);
        m_elevator.config_kP(0, ClimberConstants.kElevatorkP);
        m_elevator.config_kD(0, ClimberConstants.kElevatorkD);

        m_elevatorMotorSim = new WPI_TalonSRX(ClimberConstants.kElevatorTalonID);
        m_elevatorSimData = m_elevator.getSimCollection();

        m_elevatorSim = new ElevatorSim(
            DCMotor.getFalcon500(1),
            ClimberConstants.kElevatorGearing,
            ClimberConstants.kElevatorCarriageMass,
            ClimberConstants.kElevatorDrumRadius,
            ClimberConstants.kElevatorTicksMin,
            ClimberConstants.kElevatorTicksMax,
            null
        );

        m_mech2D = new Mechanism2d(
            ClimberConstants.kElevatorWidth, 
            ClimberConstants.kElevatorHeight
        );

        m_mechRoot2D = m_mech2D.getRoot(
            "Elevator Root", 
            ClimberConstants.kElevatorRootX,
            ClimberConstants.kElevatorRootY
        );

        m_elevatorMech2D = m_mechRoot2D.append(
            new MechanismLigament2d(
                "Elevator", 
                Units.metersToInches(m_elevatorSim.getPositionMeters()), 
                90)
        );
    }

    public void initDefaultCommand() { 
        setDefaultCommand(new InstantCommand(() -> 
        elevatorDefault()));
    }
    
    public void elevatorDefault() {
        m_elevator.set(ControlMode.PercentOutput, 0);
    }
    
    public void moveElevator(double speed) {
        m_elevator.set(ControlMode.PercentOutput, speed);
    }

    public void moveElevatorUp() {
        if (m_elevator.getSelectedSensorPosition() < ClimberConstants.kElevatorTicksUp) {
            m_elevator.set(ControlMode.PercentOutput, 0.32);
            SmartDashboard.putString(" Running Command ", "Elevator Up ");
        } else if (m_elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksUp) {
            m_elevator.set(ControlMode.PercentOutput, 0);
        }
    }

    public void moveElevatorExtend() {
        if (m_elevator.getSelectedSensorPosition() < ClimberConstants.kElevatorTicksExtend) {
            m_elevator.set(ControlMode.PercentOutput, 0.32);
            SmartDashboard.putString(" Running Command ", "Elevator Up Extend ");
        } else if (m_elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksExtend) {
            m_elevator.set(ControlMode.PercentOutput, 0);
        }
    }

    public void moveElevatorDown() {
        if (m_elevator.getSelectedSensorPosition() > ClimberConstants.kElevatorTicksDown ){
            m_elevator.set(ControlMode.PercentOutput, -0.4);
            SmartDashboard.putString(" Running Command ", "Elevator Down ");
        } else if (m_elevator.getSelectedSensorPosition() <= ClimberConstants.kElevatorTicksDown) {
            m_elevator.set(ControlMode.PercentOutput, 0);
        }
    }

    public void setPositionMotionMagic(double ticks){
        m_elevator.set(ControlMode.MotionMagic, ticks);
    }

    public void resetElevatorEncoder() {
        m_elevator.setSelectedSensorPosition(0);
        m_elevatorMotorSim.setSelectedSensorPosition(0);
    }

    public void setNeutralModeBrake() {
        m_elevator.setNeutralMode(NeutralMode.Brake);
    }
    
    public void setNeutralModeCoast() {
        m_elevator.setNeutralMode(NeutralMode.Coast);
    }

    public void reportToSmartDashboard(){
        SmartDashboard.putNumber("Elevator Position", m_elevator.getSelectedSensorPosition());
        SmartDashboard.putNumber("Elevator Velocity", m_elevator.getSelectedSensorVelocity());

        SmartDashboard.putData(" Elevator Sim ", m_mech2D);
    }

    public void log() {
        Log.createTopic("Elevator Position" + ("/Position"), () -> m_elevator.getSelectedSensorPosition());
        Log.createTopic("Elevator Velocity" + ("/Velocity"), () -> m_elevator.getSelectedSensorVelocity());
    }

    public void simPeriodic() {
        m_elevatorSim.setInput(m_elevatorMotorSim.getMotorOutputVoltage());
        m_elevatorSim.update(0.02);

        m_elevatorSimData.setQuadratureVelocity((int) (m_elevatorSim.getVelocityMetersPerSecond()));
        m_elevatorSimData.setQuadratureRawPosition((int) (m_elevatorSim.getPositionMeters()));

        m_elevatorMotorSim.setSelectedSensorPosition(m_elevatorSim.getPositionMeters());

        m_elevatorMech2D.setLength(Units.metersToInches(m_elevatorSim.getPositionMeters()));
        
    }
    
}