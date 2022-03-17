package frc.robot;

public class Constants {
    public static final class ClimberConstants {
        public static final double kArmkP = 0.04;
        public static final double kArmkI = 0;
        public static final double kArmkD = 0.4;
        public static final double kArmkF = 0; 

        public static final double kArmStaticGain = 0.0;
        public static final double kArmGravityGain = 0.11;
        public static final double kArmVelocityGain = 0;

        public static final double kArmGravityFF = 0.11;
        public static final double kArmStaticFF = 0.0;

        public static final double kArmCruiseVelocity = 200;
        public static final double kArmMotionAcceleration = 100;
        public static final double kArmMaxVelocity = 540;
        public static final double kArmMaxAcceleration = 250;
        public static final double kArmDeadband = 0.004;
        public static final double kArmStaticFrictionDeadband = 5.0;

        public static final double kArmAngleOffset = 42;
        public static final double kArmAngleLength = 1.0 / 4096 * 360 * 29;

        public static final double kTicksToRungAngle = 200; //Actual ticks 620, not resetting encoder position properly
        public static final double kTicksToVertical = -60;
    }
}
