// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;



public class Limelight{
  private NetworkTable table;
  private final int[] VALID_IDS = {11};

  public Limelight(){
    this.table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public boolean isValidID(double id){
    for (int n : VALID_IDS){
      if (n == id)
        return true;
    }
    return false;
  }

  public boolean seesValidTag(){
    if (table.getEntry("tv").getDouble(0) == 1)
      return isValidID(table.getEntry("tid").getDouble(0));
    return false;
  }

  public double[] poseRelativeToValidTag(){
    System.out.println(table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]));
    return table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
  }

  

}
