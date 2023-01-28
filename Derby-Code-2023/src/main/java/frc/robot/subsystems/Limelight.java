// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.reflect.Array;
import java.security.Key;
import java.util.ArrayList;

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
  private ShuffleboardTab position = Shuffleboard.getTab("Robot-Position");
  private final GenericEntry positionTab; 
  private double[] robotPosition; 

  public Limelight(){
    //networktables
    NetworkTableInstance.getDefault().startServer();
    NetworkTableInstance.getDefault().setServerTeam(5409);
    
    positionTab = position.add("x,y,z,rx,ry,rz", 0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double[] fieldSpacePosition(){
    //get the position of the robot in 3d fieldspace as calculated by fiducial targets
    double[] robotPos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("t6r_fs").getDoubleArray(0); //TEMPORARY
    positionTab.setDouble(robotPos[0]); //testing
    return robotPos;
  }
}
