package frc.robot.autos;


import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.commands.AlignTag;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PizzaBox;
import frc.robot.subsystems.Arm;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public final class AutoRoutines{
        public static final class Amp{
            public static final Pose2d[] RED_LEFT = null;
            public static final Pose2d[] RED_CENTER = null;
            public static final Pose2d[] RED_RIGHT = null;
            public static final Pose2d[] BLUE_LEFT = null;
            public static final Pose2d[] BLUE_CENTER = null;
            public static final Pose2d[] BLUE_RIGHT = null;

            public static SequentialCommandGroup routine(CommandSwerveDrivetrain drivetrain, Arm arm, PizzaBox pizzaBox, Limelight limelight){
            return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.tareEverything(), drivetrain).withTimeout(1),
            new ParallelCommandGroup(
                new AlignTag(drivetrain, limelight, AlignTag.NO_CHANGE, -10), new InstantCommand(() -> arm.moveToPose(Constants.ArmPositions.AMP))
            ),
            new AlignTag(drivetrain, limelight, -20, AlignTag.NO_CHANGE),
            pizzaBox.shoot()
            );
        }
        }
}