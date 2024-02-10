package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    public static final double stickDeadband = 0.05;

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI / 4;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPX = .5;
        public static final double kPY = .5;
        public static final double kIX = 0;
        public static final double kIY = 0;
        public static final double kDX = 0;
        public static final double kDY = 0;
        public static final double kPTheta = 0.5;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class ArmConstants {
        public static final int rotationMotorID = 16;
        public static final double rotationSpeed = 0.01;
        public static double minEncoderValue = 0;
        public static double maxEncoderValue = 1;
        public static final double encoderDifference = 0;
    }

    public static final class ExtenderConstants {
        public static final int extensionMotorID = 17;
    }

    public static final class ClawConstants {
        public static final int claw1ID = 15;
        public static final int claw2ID = 14;

        public static final double clawVoltage = 12.0;
    }

    public static final class Balance { // TODO move to balance command
        public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
        public static final double BEAM_BALANACED_DRIVE_KP = 1;
        public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 3;
        public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
    }
}
