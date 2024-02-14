// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.wrappers.MonitoredPIDController;

public class AutoDriveCommand extends Command {
  private final Timer timer = new Timer();
  private final Trajectory trajectory;
  private final HolonomicDriveController controller;
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.MaxSpeed * .1).withRotationalDeadband(Constants.MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AutoDriveCommand(CommandSwerveDrivetrain drivetrain, Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    ProfiledPIDController thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPTheta, 0 , 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    MonitoredPIDController xController = new MonitoredPIDController(Constants.AutoConstants.kPX, Constants.AutoConstants.kIX, Constants.AutoConstants.kDX, "X PID Controller");
    MonitoredPIDController yController = new MonitoredPIDController(Constants.AutoConstants.kPY, Constants.AutoConstants.kIY, Constants.AutoConstants.kDY, "Y PID Controller");

    this.trajectory = trajectory;
    this.controller = new HolonomicDriveController(
      requireNonNullParam(xController, "xController", "SwerveControllerCommand"),
      requireNonNullParam(yController, "yController", "SwerveControllerCommand"),
      requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand"));
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > 2){
    double curTime = timer.get() - 2;
    System.out.println(curTime);
    var desiredState = trajectory.sample(curTime);
    var desiredRotation = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation();

    ChassisSpeeds targetChassisSpeeds = controller.calculate(drivetrain.getCachedPose(), desiredState, desiredRotation);
    drivetrain.applyRequest(() -> drive.withVelocityX(targetChassisSpeeds.vxMetersPerSecond) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(targetChassisSpeeds.vyMetersPerSecond) // Drive left with negative X (left)
            .withRotationalRate(targetChassisSpeeds.omegaRadiansPerSecond) // Drive counterclockwise with negative X (left)
        );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
