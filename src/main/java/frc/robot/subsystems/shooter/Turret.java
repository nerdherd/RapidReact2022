package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.TurnToTarget;
import frc.robot.subsystems.Limelight;
import frc.robot.util.NerdyMath;

public class Turret extends SubsystemBase {
    
    private TalonFX frontFlywheelFalcon;
    private TalonFX backFlywheelFalcon;
    private TalonSRX hoodMotor;
    private TalonSRX baseMotor;
    private TalonSRX feederMotor;

    private TurnToTarget turnToTargetCommand;

    private boolean feederIsRunning;
    private boolean tarmacOutsideIsRunning;
    private boolean tarmacInsideIsRunning;
    
    public Turret(Limelight limelight) {
        feederIsRunning = false;
        tarmacInsideIsRunning = false;
        tarmacOutsideIsRunning = false;

        this.frontFlywheelFalcon = new TalonFX(TurretConstants.kFrontFlywheelFalconID);
        this.backFlywheelFalcon = new TalonFX(TurretConstants.kBackFlywheelFalconID);
        this.hoodMotor = new TalonSRX(TurretConstants.kHoodMotorID);
        this.baseMotor = new TalonSRX(TurretConstants.kBaseMotorID);
        this.feederMotor = new TalonSRX(TurretConstants.kFeederMotorID);

        this.hoodMotor.config_kP(0, TurretConstants.kHoodP);
        this.hoodMotor.config_kI(0, TurretConstants.kHoodI);
        this.hoodMotor.config_kD(0, TurretConstants.kHoodD);
        this.hoodMotor.config_kF(0, TurretConstants.kHoodF);
        this.hoodMotor.configMotionAcceleration(TurretConstants.kHoodAcceleration);
        this.hoodMotor.configMotionCruiseVelocity(TurretConstants.kHoodCruiseVelocity);
        this.hoodMotor.set(ControlMode.Current, 0);
        this.hoodMotor.configNeutralDeadband(TurretConstants.kHoodDeadband);
        this.baseMotor.config_kP(0, TurretConstants.kBaseP);
        this.baseMotor.configMotionAcceleration(TurretConstants.kBaseMotionAcceleration);
        this.baseMotor.configMotionCruiseVelocity(TurretConstants.kBaseCruiseVelocity);
        this.baseMotor.setInverted(true);

        this.frontFlywheelFalcon.set(ControlMode.PercentOutput, 0);
        this.backFlywheelFalcon.set(ControlMode.PercentOutput, 0);

        this.turnToTargetCommand = new TurnToTarget(this, limelight);
        setDefaultCommand(this.turnToTargetCommand);
    }

    public void toggleHood() {
        this.turnToTargetCommand.toggleHood();
    }

    public void setPercent(double power) {
        backFlywheelFalcon.set(ControlMode.PercentOutput, power * TurretConstants.kBackFlywheelGearRatio);    
        frontFlywheelFalcon.set(ControlMode.PercentOutput, power);
    }

    public void setPercentZero() {
        setPercent(0);
    }

    public void setFeederPercent(double power) {
        feederMotor.set(ControlMode.PercentOutput, power);
    }

    public void setFeederPercentZero(double power) {
        setFeederPercent(0);
    }

    public void toggleFlywheelOutside() {
        if (tarmacOutsideIsRunning) {
            setPercentZero();
            tarmacOutsideIsRunning = false;
        } else {
            setPercent(TurretConstants.kFlywheelOuterTarmacPercent);
            tarmacOutsideIsRunning = true;
        }
        tarmacInsideIsRunning = false;
    }

    public void toggleFlywheelInside() {
        if (tarmacInsideIsRunning) {
            setPercentZero();
            tarmacInsideIsRunning = false;
        } else {
            setPercent(TurretConstants.kFlywheelInnerTarmacPercent);
            tarmacInsideIsRunning = true;
        }
        tarmacOutsideIsRunning = false;
    }

    public void toggleFeeder() {
        if (feederIsRunning) {
            setPercentZero();
            feederIsRunning = false;
        } else {
            setPercent(TurretConstants.kfeederPercent);
            feederIsRunning = true;
        }
    }

    // public void setVelocity(double velocity) {
    //     backFlywheelFalcon.set(ControlMode.Velocity, velocity);
        
    //     // Get front flywheel to spin at 1/4 the speed
    //     frontFlywheelFalcon.set(ControlMode.Velocity, velocity 
    //                             * (1/TurretConstants.kBackFlywheelGearRatio) 
    //                             * 1);
    // }

    // public void setVelocityZero() {
    //     setVelocity(0);
    // }

    public void turnToBaseAngle(double angle) {
        double currentAngle = getCurrentBaseAngle();
        double offsetAngle = constrainAngleBase(angle);
        double targetAngle = currentAngle + offsetAngle;
        double targetPosition = TurretConstants.kBaseTicksPerDegree * targetAngle;
        baseMotor.set(ControlMode.MotionMagic, targetPosition);
    }

    public double getCurrentBaseAngle() {
        return baseMotor.getSelectedSensorPosition() / TurretConstants.kBaseTicksPerDegree;
    }

    public double getCurrentHoodAngle() {
        return hoodMotor.getSelectedSensorPosition() / TurretConstants.kHoodTicksPerDegree;
    }

    public void turnToHoodTicks(double ticks) {
        hoodMotor.set(
                    ControlMode.MotionMagic, 
                    NerdyMath.clamp(ticks, 
                        TurretConstants.kHoodLowerLimitTicks, 
                        TurretConstants.kHoodUpperLimitTicks));
    }

    public void turnToHoodAngle(double angle) {
        turnToHoodTicks(
            TurretConstants.kHoodTicksPerDegree * angle
        );
    }

    private double constrainAngleBase(double rawAngle) {
        double newAngle = rawAngle % 360;
        if (newAngle <= 0) {
            newAngle += 360;
        }

        return newAngle - 180;
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber("Hood current", getHoodCurrent());
        SmartDashboard.putNumber("Hood ticks", hoodMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Hood angle", getCurrentHoodAngle());
        SmartDashboard.putNumber("Base current", getBaseCurrent());
        SmartDashboard.putNumber("Base ticks", baseMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Base angle", getCurrentBaseAngle());
    }

    public double getHoodCurrent() {
        return this.hoodMotor.getStatorCurrent();
    }

    public double getBaseCurrent() {
        return this.baseMotor.getStatorCurrent();
    }

    public void resetHoodEncoder() {
        this.hoodMotor.setSelectedSensorPosition(0);
    }

}