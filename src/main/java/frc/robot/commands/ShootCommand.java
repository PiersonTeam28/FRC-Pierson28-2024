// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PizzaBox;

public class ShootCommand extends Command {
  /** Creates a new CloseClaw. */
  private final PizzaBox pizzaBox;
  private final Timer timer = new Timer();

  public ShootCommand(PizzaBox pizzaBox) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pizzaBox = pizzaBox;
    addRequirements(this.pizzaBox);
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("start shooting");
    timer.restart();
    this.pizzaBox.startShootMotors();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(timer.get());
    if (timer.get() >= 1.5)
      this.pizzaBox.releaseHoldingMotor();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.pizzaBox.stopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= 2;
  }
}
