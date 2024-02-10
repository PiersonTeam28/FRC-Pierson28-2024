// package frc.robot.autos;

// import frc.robot.Constants;
// import frc.robot.subsystems.Swerve;

// import java.util.Arrays;
// import java.util.List;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.util.datalog.*;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.util.sendable.SendableRegistry;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

// public class Auto extends SequentialCommandGroup {
//     Swerve s_Swerve;

//     public Auto(Swerve s_Swerve){
//         this.s_Swerve = s_Swerve;

//         addCommands(
//         getDriveCommand(    new Pose2d(0, 0, new Rotation2d(0)), 
//                             new Pose2d(5, 0, new Rotation2d(0)),
//                             new Pose2d(3, 3, new Rotation2d(0))
//                             //new Pose2d(0, 3, new Rotation2d(0))
//                             //new Pose2d(0, 0, new Rotation2d(0))
//                             //new Pose2d(0, .1, new Rotation2d(0))
//                         ),
//         new InstantCommand(() -> s_Swerve.stop())
//         );
//     }

//     public SwerveControllerCommand getDriveCommand(Pose2d...waypoints){
//         TrajectoryConfig config =
//             new TrajectoryConfig(
//                     Constants.AutoConstants.kMaxSpeedMetersPerSecond,
//                     Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                 .setKinematics(Constants.Swerve.swerveKinematics);

//         // An example trajectory to follow.  All units in meters.
//         //Pose2d[] waypoints = {startPose, endPose};
//         Trajectory exampleTrajectory =
//             TrajectoryGenerator.generateTrajectory(
//                 Arrays.asList(waypoints),
//                 // Arrays.asList(intermediatePoints), //new Translation2d(-1, 1), new Translation2d(-1, 0)),
//                 config);

//         s_Swerve.resetOdometry(exampleTrajectory.getInitialPose());

//         var thetaController =
//             new ProfiledPIDController(
//                 Constants.AutoConstants.kPTheta, 0 , 0, Constants.AutoConstants.kThetaControllerConstraints);
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         MonitoredPIDController xController = new MonitoredPIDController(Constants.AutoConstants.kPX, Constants.AutoConstants.kIX, Constants.AutoConstants.kDX, "X PID Controller");
//         MonitoredPIDController yController = new MonitoredPIDController(Constants.AutoConstants.kPY, Constants.AutoConstants.kIY, Constants.AutoConstants.kDY, "Y PID Controller");
//         SendableRegistry.add(xController, "X PID");
//         SendableRegistry.add(yController, "Y PID");
//         SmartDashboard.putData(xController);
//         SmartDashboard.putData(yController);
//         System.out.print("Smart Dashboard keys: " + SmartDashboard.getData("X PID"));
        
//         SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//                 exampleTrajectory,
//                 s_Swerve::getPose,
//                 Constants.Swerve.swerveKinematics,
//                 xController,
//                 yController,
//                 thetaController,
//                 s_Swerve::setModuleStates,
//                 s_Swerve);
//         return swerveControllerCommand;
//     }
// }