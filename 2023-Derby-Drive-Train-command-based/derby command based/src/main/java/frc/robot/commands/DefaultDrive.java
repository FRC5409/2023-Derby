// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


/** An example command that uses an example subsystem. */
public class DefaultDrive extends CommandBase {
private DriveTrain sys_drive;
private XboxController joystick;

  //  private final DefaultDrive sys_drive;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param m_subsystem 
   */
  public DefaultDrive(DriveTrain driveTrain, XboxController joystick) {
    sys_drive = driveTrain;
    
    
    // Use addRequirements() here to declare subsystem dependencies.
//    addRequirements(DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Left trigger(acceleration), right trigger(deceleration), LeftX
    double fAccel = joystick.getLeftTriggerAxis();
    double rAccel = joystick.getRightTriggerAxis();
    double turnVal = joystick.getLeftX();

    sys_drive.arcadeDrive(fAccel - rAccel, turnVal);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
