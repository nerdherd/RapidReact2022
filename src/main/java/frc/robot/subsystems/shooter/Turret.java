package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NerdyMath;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.TurnToTarget;

public class Turret extends SubsystemBase {
    
    private TalonFX frontFlywheelFalcon;
    private TalonFX backFlywheelFalcon;
    private TalonFX hoodMotor;
    private TalonFX baseMotor;

    private TurnToTarget turnToTargetCommand;

    private double hoodLimitLower, hoodLimitUpper;

    public Turret() {
        this.frontFlywheelFalcon = new TalonFX(TurretConstants.kFrontFlywheelFalconID);
        this.backFlywheelFalcon = new TalonFX(TurretConstants.kBackFlywheelFalconID);
        this.hoodMotor = new TalonFX(TurretConstants.kHoodMotorID);
        this.baseMotor = new TalonFX(TurretConstants.kBaseMotorID);

        hoodLimitLower = NerdyMath.ticksToAngle(TurretConstants.kHoodLowerLimitTicks);
        hoodLimitUpper = NerdyMath.ticksToAngle(TurretConstants.kHoodUpperLimitTicks);

        this.frontFlywheelFalcon.set(ControlMode.PercentOutput, 0);
        this.backFlywheelFalcon.set(ControlMode.PercentOutput, 0);

    }

    public void setTurnToTargetCommand(TurnToTarget command) {
        this.turnToTargetCommand = command;
        setDefaultCommand(this.turnToTargetCommand);
    }

    public void toggleHood() {
        this.turnToTargetCommand.toggleHood();
    }

    public void setPower(double power) {
        backFlywheelFalcon.set(ControlMode.PercentOutput, power);
        
        // Get front flywheel to spin at 1/4 the speed
        frontFlywheelFalcon.set(ControlMode.PercentOutput, power 
                                * (1/TurretConstants.kBackFlywheelGearRatio) 
                                * 1);
    }

    public void setVelocity(double velocity) {
        backFlywheelFalcon.set(ControlMode.Velocity, velocity);
        
        // Get front flywheel to spin at 1/4 the speed
        frontFlywheelFalcon.set(ControlMode.Velocity, velocity 
                                * (1/TurretConstants.kBackFlywheelGearRatio) 
                                * 1);
    }

    public void setVelocityZero() {
        setVelocity(0);
    }

    public void turnToBaseAngle(double angle) {
        double currentAngle = getCurrentBaseAngle();
        double offsetAngle = TurretConstants.kBaseTicksPerDegree * constrainAngleBase(angle);
        double targetAngle = currentAngle + offsetAngle;
        baseMotor.set(ControlMode.MotionMagic, targetAngle);
    }

    public double getCurrentBaseAngle() {
        return baseMotor.getSelectedSensorPosition() / TurretConstants.kBaseTicksPerDegree;
    }

    public double getCurrentHoodAngle() {
        return hoodMotor.getSelectedSensorPosition() / TurretConstants.kHoodTicksPerDegree;
    }

    public void turnToHoodAngle(double angle) {
        hoodMotor.set(
            ControlMode.MotionMagic, 
            TurretConstants.kHoodTicksPerDegree * 
            NerdyMath.clamp(angle, hoodLimitLower, hoodLimitUpper)
            );
    }

    private double constrainAngleBase(double rawAngle) {
        double newAngle = rawAngle % 360;
        if (newAngle < 0) {
            newAngle += 360;
        }

        return newAngle - 180;
    }

}