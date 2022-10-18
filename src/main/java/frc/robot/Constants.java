package frc.robot;

public final class Constants {

    public static final class DriveConstants {
        public static final double kDriveAlpha = 0.11765;
        public static final double kDriveOneMinusAlpha = 0.88235;

        public static final int kRightMasterTalonID = 30;
        public static final int kRightSlaveTalonID = 31;
        public static final int kLeftMasterTalonID = 16;
        public static final int kLeftSlaveTalonID = 17;

        public static final int kHooksMasterTalonID = 20; //new
        public static final int kHooksSlaveTalonID = 27; //new

        public static final int kDriveShifterForwardID = 0;
        public static final int kDriveShifterReverseID = 7;
        public static final int kClimberShifterForwardID = 2;
        public static final int kClimberShifterReverseID = 3;
        public static final int kHookShifterForwardID = 1;
        public static final int kHookShifterReverseID = 6;

    }

    public static final class EverybotConstants {
        public static final double kHighAngle = -20073.00;
        public static final double kLowAngle = 0;
        public static final double kGravityAngle = -19500.00;
        public static final double kHighAngleThreshold = 0;
        public static final double kLowAngleThreshold = 0;

        public static final double kEverybotUpVoltage = 0.7;
        public static final double kEverybotDownVoltage = 0.2;
    
        public static final double kEverybotIntake = 0.5;
        public static final double kEverybotOuttake = 0.6;
        public static final double kEverybotAutoOuttake = 1.0;
    
        public static final double kEverybotClimberUp = 0.2;
        public static final double kEverybotClimberDown = 0.5;
        public static final double kEverybotClimberHigh = 0;
        public static final double kEverybotClimberLow = 0;
        
        public static final double kEverybotClimberkP = 0.02; //0.01
        public static final double kEverybotClimberkI = 0;
        public static final double kEverybotClimberkD = 0;
        public static final double kEverybotClimberkF = 0.094;
        
        public static final double kTicksToLowRung = 17826; //-102862 // 32645;
        public static final double kTicksToClimbLowRung = 41525 + 4000  - kTicksToLowRung;
        public static final double kTicksToHome = 43757;
    }
    
    public static final class ClimberConstants {
        public static final int kArmTalonID = 15;
        public static final int kElevatorTalonID = 7;

        public static final double kArmkP = 2.0; //0.5
        public static final double kArmkI = 0;
        public static final double kArmkD = 0.6; //0.6 
        public static final double kArmkF = 2.82; // 2.82

        public static final double kArmGravityFF = 0.41; //0.11 voltage calc, 0.41 phys calc

        public static final double kArmStaticFF = 0.0;

        public static final double kArmCruiseVelocity = 250;
        public static final double kArmMotionAcceleration = 125;
        public static final double kArmMaxVelocity = 575;
        public static final double kArmMaxAcceleration = 575;
        public static final double kArmDeadband = 0.004;
        public static final double kArmStaticFrictionDeadband = 5.0;

        public static final double kArmAngleOffset = 42;
        public static final double kArmAngleLength = 1.0 / 4096 * 360 * 29;

        public static final double kTicksToRungAngle = 200; //Actual ticks 620, not resetting encoder position properly
        public static final double kTicksToClearRung = 250;
        public static final double kTicksToVertical = 60;
        public static final double kArmTicksDownSoftLimit = 330;
        public static final double kArmTicksUpSoftLimit = 0;
        public static final double kElevatorTicksUp = 22000;
        public static final double kElevatorTicksExtend = 23000;
        public static final double kElevatorTicksDown = 2000;

        public static final double kSoftLimitTicks = 20000;

        public static final double kOperatorDeadband = 0.007874;
        public static final double kOperatorAlpha = 0.11765;
        public static final double kOperatorOneMinusAlpha = 0.88235;
    }

    public static final class FlywheelConstants {
        public static final double kMotionMagicCruiseVelocity = 10888.53;
        public static final double kMotionMagicAcceleration = 10888.53;
        public static final double kFlywheelP = 0;
        public static final double kFlywheelI = 0;
        public static final double kFlywheelD = 0;
        public static final double kFlywheelFF = 0;
        public static final int kRightFlywheelID = 35;
        public static final int kLeftFlywheelID = 34;
        public static final double kFlywheelVelocity = 0;
        public static final double kFlywheelPercent = 0.55; 
    }

    public static final class IndexerConstants {
        public static final int kIndexerTopID = 20;
        public static final int kIndexerBottomID = 21;
        public static final double kIndexerPercent = .70;
    }

    public static final class RollerConstants {
        public static final int kRollerID = 38;
        public static final double kRollerPercent = -0.95;
    }

    public static final class IntakeConstants{
        public static final int kIntakeID = 37;
        public static final double kIntakeUpPosition = -2714; // When ready to intake
        public static final double kIntakeOffset = -4189; // Ticks when perpendicular to ground
        public static final double kIntakeGravityFF = 0.175; // Negative goes up, should invert
        public static final double kIntakeDownPosition = -1715; // 1098
        public static final double kIntakeMotionAcceleration = 125;
        public static final double kIntakeCruiseVelocity = 250;
        public static final double kIntakeDeadband = 0.004;
        public static final double kIntakeP = 2.2; // 2
        public static final double kIntakeD = 0;
        public static final double kIntakeF = 0; // 2.8

    }

    public static final class TurretConstants {
        public static final int kFrontFlywheelFalconID = 18;
        public static final int kBackFlywheelFalconID = 3;
        public static final int kHoodMotorID = 36;
        public static final int kBaseMotorID = 0;

        public static final double kHoodF = 0.867;
        public static final double kHoodP = 0.1;
        public static final double kHoodD = 0.6;
        public static final double kHoodI = 0;

        public static final double kHoodCruiseVelocity = 100;
        public static final double kHoodAcceleration = 50;
        public static final double kHoodMaxVelocity = 200;
        public static final double kHoodMaxAccel = 100;
        public static final double kHoodDeadband = 0.07; // One Degree 


        public static final double kBaseGearRatio = 0;
        public static final double kBackFlywheelGearRatio = 39 / 30.0;

        public static final double kBaseTicksPerDegree = 0;
        public static final double kHoodTicksPerDegree = 238.3;

        // NOTE: Hood angle is measured from hood axle to axle of upper flywheel
        public static final double kHoodLowerLimitTicks = 0; // Ticks when at 15 degrees should ideally be 25700 ticks, 0 ticks is parallel to ground
        public static final double kHoodUpperLimitTicks = 7648;
    }

    public static final class VisionConstants {
        public static final int kTargetHeight = 0;
        public static final int kCameraMountHeight = 0;
        public static final int kTargetWidth = 0;
        public static final int kXFocalLength_lime = 0;
        public static final double kCameraBumperOffset = 0;
    }

}
