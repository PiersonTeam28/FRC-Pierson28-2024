// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.wrappers.MonitoredPIDController;

public class AlignTag extends Command {
  public static final double NO_CHANGE = 100;

  private CommandSwerveDrivetrain drivetrain;
  private Limelight limelight;
  private MonitoredPIDController xController;
  private MonitoredPIDController yController;
  private int alliance;

  public AlignTag(CommandSwerveDrivetrain drivetrain, Limelight limelight, double desiredHorizontalOffset, double desiredVerticalOffset, int alliance) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.alliance = alliance;
    xController = new MonitoredPIDController(.2, 0, 0, "X Controller Align");
    xController.setSetpoint(desiredHorizontalOffset);
    xController.setTolerance(.2);
    yController = new MonitoredPIDController(.1, .1, .2, "Y Controller Align");
    yController.setSetpoint(desiredVerticalOffset);
    yController.setTolerance(.1);
    if (desiredHorizontalOffset == NO_CHANGE)
      xController.disable();
    if (desiredVerticalOffset == NO_CHANGE)
      yController.disable();
  }

  @Override
  public void initialize() {
    drivetrain.setControl(Constants.drive.withVelocityY(alliance == 0 ? -1 : 1));
  }

  @Override
  public void execute() {
    if (limelight.seesValidTag()){
      drivetrain.setControl(
        Constants.drive
        .withVelocityY(MathUtil.clamp(yController.calculate(limelight.getHorizontalOffset()), -1, 1))
        .withVelocityX(MathUtil.clamp(xController.calculate(limelight.getVerticalOffset()), -1, 1))
      );
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("end");
    drivetrain.setControl(Constants.brake);
  }

  @Override
  public boolean isFinished() {
    return xController.isDisabled() ? yController.atSetpoint() : 
    yController.isDisabled() ? xController.atSetpoint() : 
    xController.atSetpoint() && yController.atSetpoint();
  }
}
