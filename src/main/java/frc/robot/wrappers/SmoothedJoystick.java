// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public final class SmoothedJoystick extends CommandJoystick {
private double smoothConstant;

  public SmoothedJoystick(int channel, double smoothConstant) {
    super(channel);
    this.smoothConstant = smoothConstant;
  }

  @Override
  public double getMagnitude(){
    double magnitude = super.getMagnitude();
    return mapValue(magnitude);
  }

  @Override
  public double getX(){
    return getMagnitude() * Math.cos(getDirectionRadians());
  }

  @Override
  public double getY(){
    return getMagnitude() * Math.sin(getDirectionRadians());
  }

  @Override
  public double getTwist(){
    return mapValue(super.getTwist());
  }

  private double mapValue(double rawValue){
    if (rawValue < 0){
      rawValue *= -1;
      rawValue = Math.pow(rawValue, smoothConstant);
      return rawValue * -1;
    }
    else{
      return Math.pow(rawValue, smoothConstant);
    }
  }
}
