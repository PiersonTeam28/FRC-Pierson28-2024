package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Arrays;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto extends SequentialCommandGroup {

    public Auto(CommandSwerveDrivetrain drivetrain){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(drivetrain.getKinematics());

        Trajectory trajectory_1 =
            TrajectoryGenerator.generateTrajectory(
                Arrays.asList(new Pose2d(new Translation2d(1, 0), new Rotation2d(Math.PI))),
                config);
        addCommands(
            new AutoDriveCommand(drivetrain, trajectory_1)
        );
    }
}