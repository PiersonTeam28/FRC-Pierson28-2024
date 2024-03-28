// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PizzaBox;
import frc.robot.wrappers.SmoothedJoystick;
import frc.robot.autos.AutoRoutines;

public class RobotContainer {
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SmoothedJoystick joystick = new SmoothedJoystick(0, 1.25); // My joystick
  private final CommandXboxController xboxController = new CommandXboxController(1);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
                                                               // driving in open loop
  private final Arm arm = new Arm();
  private final PizzaBox pizza = new PizzaBox();
  private final Limelight limelight = new Limelight();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(Constants.MaxSpeed);
  
  private int alliance;

  private void configureBindings(int alliance) {
    this.alliance = alliance;
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> Constants.drive.withVelocityX(-joystick.getY() * Constants.MaxSpeed * Math.pow(-1, alliance)) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getX() * Constants.MaxSpeed * Math.pow(-1, alliance)) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getTwist() * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.button(9).whileTrue(drivetrain.applyRequest(() -> Constants.brake));
    // joystick.button(7).whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getY(), -joystick.getX()))));
    // joystick.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    joystick.button(5).onTrue(new InstantCommand(() -> arm.moveToPose(Constants.ArmPositions.STOW)));
    joystick.button(3).onTrue(new InstantCommand(() -> arm.moveToPose(Constants.ArmPositions.SPEAKER)));
    joystick.button(4).onTrue(new InstantCommand(() -> arm.moveToPose(Constants.ArmPositions.SOURCE)));
    joystick.button(6).onTrue(new InstantCommand(() -> arm.moveToPose(Constants.ArmPositions.AMP)));
    joystick.trigger().onTrue(pizza.shoot());
    joystick.button(2).onTrue(pizza.intake()).onFalse(pizza.stop());
    joystick.povUp().onTrue(arm.up(joystick.povUp()));
    joystick.povDown().onTrue(arm.down(joystick.povDown()));

    xboxController.rightTrigger().onTrue(pizza.dispense()).onFalse(pizza.stop());
    xboxController.leftTrigger().onTrue(pizza.intake()).onFalse(pizza.stop());
    xboxController.rightBumper().onTrue(arm.up(xboxController.rightBumper()));
    xboxController.leftBumper().onTrue(arm.down(xboxController.leftBumper()));
    xboxController.a().onTrue(new InstantCommand(() -> arm.moveToPose(Constants.ArmPositions.STOW)));
    xboxController.x().onTrue(new InstantCommand(() -> arm.moveToPose(Constants.ArmPositions.SOURCE)));
    xboxController.y().onTrue(new InstantCommand(() -> arm.moveToPose(Constants.ArmPositions.AMP)));


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer(int alliance) {
    configureBindings(alliance);
    drivetrain.runOnce(() -> drivetrain.seedFieldRelative());
  }

  public Command getAutonomousCommand() {
    drivetrain.runOnce(() -> drivetrain.seedFieldRelative());
    return new InstantCommand(() -> arm.moveToPose(Constants.ArmPositions.AMP));}
}
