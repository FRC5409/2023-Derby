// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.reflect.Array;
import java.security.Key;
import java.util.ArrayList;

import javax.swing.text.Position;

import com.ctre.phoenix.CANifier.GeneralPin;

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
  private final ShuffleboardLayout localizationPos; 
  private final ShuffleboardLayout localizationRot;

  private double[] positionDefaults = new double[]{};

  public Limelight(){
    //networktables
    NetworkTableInstance.getDefault().startServer();
    NetworkTableInstance.getDefault().setServerTeam(5409);

    //shuffleboard
    Shuffleboard.getTab("Field Localization").add("Position", 0);
    Shuffleboard.getTab("Field Localization").add("Rotation", 0);

    localizationPos = Shuffleboard.getTab("Field Localization")
        .getLayout("Position", BuiltInLayouts.kGrid)
        .withSize(1, 2);
    
    localizationPos.add("X", 0);
    localizationPos.add("Y", 0);
    localizationPos.add("Z", 0);

    localizationRot = Shuffleboard.getTab("Field Localization")
        .getLayout("Rotation", BuiltInLayouts.kGrid)
        .withSize(1, 2);
      
    localizationRot.add("rX", 0);
    localizationRot.add("rY", 0);
    localizationRot.add("rZ", 0);
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
    if (robotPos != null){ 
      //update Rotation and Position here 
      localizationPos.addDouble("X", robotPos[0]);

      //MAKES EVERYTHING CRASH
      //localizationPos.add("X", robotPos[0]);
      //localizationPos.add("Y", robotPos[1]);
      //localizationPos.add("Z", robotPos[2]);

      //localizationRot.add("rX", robotPos[3]);
      //localizationRot.add("rY", robotPos[4]);
      //localizationRot.add("rZ", robotPos[5]);
    }
  }
}