// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** An example command that uses an example subsystem. */
public class DeafultDrive extends CommandBase {
//  private final DeafultDrive sys_drive;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param m_subsystem 
   */
  public DeafultDrive(DeafultDrive DriveTrain, XboxController joystick) {
//    sys_drive = DriveTrain;
    
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
//    double acceleration = m_joystick.getLeftTriggerAxis();
//    double deceleration = m_joystick.getRightTriggerAxis();

//    double turnValue = m_joystick.getLeftX();

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
