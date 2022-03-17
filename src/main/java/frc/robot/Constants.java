package frc.robot;

public class Constants {
    public static final class ClimberConstants {
        public static final double kArmkP = 0.4;
        public static final double kArmkI = 0;
        public static final double kArmkD = 4;
        public static final double kArmkF = 2.82; 

        public static final double kArmStaticGain = 0.0;
        public static final double kArmGravityGain = 0.11;
        public static final double kArmVelocityGain = 0;

        public static final double kArmGravityFF = 0.11;
        public static final double kArmStaticFF = 0.0;

        public static final double kArmCruiseVelocity = 700;
        public static final double kArmMotionAcceleration = 700;
        public static final double kArmMaxVelocity = 1000;
        public static final double kArmMaxAcceleration = 1000;
        public static final double kArmDeadband = 0.004;
        public static final double kArmStaticFrictionDeadband = 5.0;

        public static final double kArmAngleOffset = 42;
        public static final double kArmAngleLength = 1.0 / 4096 * 360 * 29;

        public static final double kTicksToRungAngle = 200; //Actual ticks 620, not resetting encoder position properly
        public static final double kTicksToVertical = -60;
    }
}
