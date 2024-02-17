package frc.robot.autos;


import frc.robot.commands.AutoDriveCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoRoutines{

        public static SequentialCommandGroup drive(CommandSwerveDrivetrain drivetrain){
            
            return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.tareEverything(), drivetrain).withTimeout(2),
            new AutoDriveCommand(drivetrain, new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
                                             new Pose2d(new Translation2d(0, 5), new Rotation2d(0)))
            );
        }
}