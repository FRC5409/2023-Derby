// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class Drivestraight extends CommandBase {
    private float driveSpeed;
    private final DriveTrain sys_drive;

    public Drivestraight(DriveTrain subsystem, XboxController joystick) {
        sys_drive = subsystem;
        addRequirements(subsystem);
    }


      // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Left trigger(acceleration), right trigger(deceleration), LeftX(turn)
    sys_drive.tankDrive(driveSpeed, driveSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      sys_drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

