package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    public static final double stickDeadband = 0.05;
    public static final double MaxSpeed = 4.5;
    public static final double MaxAngularRate = 2 * Math.PI;
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI / 4;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPX = 1;
        public static final double kPY = 1;
        public static final double kIX = .05;
        public static final double kIY = .05;
        public static final double kDX = 0;
        public static final double kDY = 0;
        public static final double kPTheta = 0.5;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class ArmPositions{
      public static final double STOW = 0;
      // All positions are based on when robot is up agains the respective field object
      public static final double SOURCE = -21;
      public static final double SPEAKER = -13.4;
      public static final double AMP = -86;
      private static double degreesToRotations(double degrees){
        // Conversion factor based on 48:1 -> 56:20 gear reduction for Falcon driving the arm (134.4 rotations of motor to 1 rotation of arm)
        return degrees * (28.0/75.0);
    }
    // 4 inch diameter of roller -> 1 rotation = 4 inches of wire
    
    }
}
