// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** An example command that uses an example subsystem. */
public class DefaultDrive extends CommandBase {
private DifferentialDrive sys_drive;
private DefaultDrive joystick;

  //  private final DefaultDrive sys_drive;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param m_subsystem 
   */
  public DefaultDrive(DifferentialDrive DriveTrain, XboxController joystick) {
    sys_drive = DriveTrain;
    
    
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
    double acceleration = joystick.getLeftTriggerAxis();
    double deceleration = joystick.getRightTriggerAxis();

    double turnValue = joystick.getLeftX();

    aadlDrive(acceleration, deceleration, turnValue);

  }

  private double getLeftX() {
    return 0;
  }

  private double getRightTriggerAxis() {
    return 0;
  }

  private double getLeftTriggerAxis() {
    return 0;
  }

  public void aadlDrive(double fAccel, double rAccel, double turnVal) {
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
