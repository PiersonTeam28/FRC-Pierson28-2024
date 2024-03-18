package frc.robot.autos;


import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.commands.AlignTag;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PizzaBox;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public final class AutoRoutines{
        public static final class Amp{
            public static final int RED_LEFT = 0;
            public static final int RED_CENTER = 1;
            public static final int RED_RIGHT = 2;
            public static final int BLUE_LEFT = 3;
            public static final int BLUE_CENTER = 4;
            public static final int BLUE_RIGHT = 5;

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