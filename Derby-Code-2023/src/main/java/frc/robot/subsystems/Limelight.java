// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.reflect.Array;
import java.security.Key;
import java.util.ArrayList;

import javax.swing.text.Position;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Limelight extends SubsystemBase {
  //networktables
  NetworkTable limelighTable;

  //shuffleboard
  private ShuffleboardTab localization = Shuffleboard.getTab("Field Localization");
  private final GenericEntry positionTab; 
  private final GenericEntry rotationTab;  
  private double[] positionDefaults = new double[]{};

  public Limelight(){
    //networktables
    NetworkTableInstance.getDefault().startServer();
    NetworkTableInstance.getDefault().setServerTeam(5409);
    
    positionTab = localization.add("Robot Position", 0).getEntry();
    rotationTab = localization.add("Robot Rotation", 0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateRobotPosition();
  }

  public void updateRobotPosition() {
    //get the position of the robot in 3d fieldspace as calculated by fiducial targets
    double[] robotPos = NetworkTableInstance.getDefault()
      .getTable("limelight")
      .getEntry("botpose")
      .getDoubleArray(positionDefaults); //TEMPORARY
    
    //pushing to shuffleboard1
    if (robotPos.length != 0){ 
      positionTab.setDouble(robotPos[0]); //testing
    }
  }
}
