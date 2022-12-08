// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDrive;
//import frc.robot.commands.DriveStraight;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final XboxController sys_controller;
  private final JoystickButton but_main_A, but_main_B, but_main_X, but_main_Y, but_main_LBumper, but_main_RBumper, but_main_LAnalog, but_main_RAnalog, but_main_Back, but_main_Start;

  private final DriveTrain sys_drive;
  
  private final DefaultDrive cmd_defaultDrive;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    sys_controller = new XboxController(0);
    but_main_A = new JoystickButton(sys_controller, XboxController.Button.kA.value);
    but_main_B = new JoystickButton(sys_controller, XboxController.Button.kB.value);
    but_main_X = new JoystickButton(sys_controller, XboxController.Button.kX.value);
    but_main_Y = new JoystickButton(sys_controller, XboxController.Button.kY.value);
    but_main_LBumper = new JoystickButton(sys_controller, XboxController.Button.kLeftBumper.value);
    but_main_RBumper = new JoystickButton(sys_controller, XboxController.Button.kRightBumper.value);
    but_main_LAnalog = new JoystickButton(sys_controller, XboxController.Button.kLeftStick.value);
    but_main_RAnalog = new JoystickButton(sys_controller, XboxController.Button.kRightStick.value);
    but_main_Back = new JoystickButton(sys_controller, XboxController.Button.kBack.value);
    but_main_Start = new JoystickButton(sys_controller, XboxController.Button.kStart.value);

    sys_drive = new DriveTrain();

    cmd_defaultDrive = new DefaultDrive(sys_drive, sys_controller);

    sys_drive.setDefaultCommand(cmd_defaultDrive);
    
    // Configure the button bindings
    configureButtonBindings();
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//  public Command getAutonomousCommand() {
//    return m_autonomousCommand;
//  }
//}

