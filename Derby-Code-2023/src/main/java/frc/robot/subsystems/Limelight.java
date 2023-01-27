// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Limelight extends SubsystemBase {
  NetworkTable limelighTable;

  public Limelight(){
    NetworkTableInstance.getDefault().startServer();
    NetworkTableInstance.getDefault().setServerTeam(5409);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
