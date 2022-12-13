// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pneumatics;

public class GearShift extends CommandBase {
  private final DriveTrain sys_driveSubsystem;
  private final boolean fast;

   //fast gearshift
    public GearShift(DriveTrain driveTrain, boolean fast) {
        sys_driveSubsystem = driveTrain;
        this.fast = fast; 
        addRequirements(sys_driveSubsystem);
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Shifts to fast gear
    if (fast)
      sys_driveSubsystem.fastShift();
    else
      sys_driveSubsystem.slowShift();
  }

  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}