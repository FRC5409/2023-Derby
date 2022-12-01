// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Auto;
import frc.robot.commands.SwitchGears;
import frc.robot.commands.TurnCompressor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_joystick;

  private final Drivetrain m_drivetrain;
  private final ArcadeDrive m_defaultDrive;
  private final Pneumatics m_pneumatics;

  private final Auto m_auto;

  private final JoystickButton bumper_left, bumper_right;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_joystick = new XboxController(0);
    m_drivetrain = new Drivetrain();
    m_defaultDrive = new ArcadeDrive(m_drivetrain, m_joystick);
    m_pneumatics = new Pneumatics();

    m_auto = new Auto(m_drivetrain);

    bumper_right = new JoystickButton(m_joystick, XboxController.Button.kRightBumper.value);
    bumper_left = new JoystickButton(m_joystick, XboxController.Button.kLeftBumper.value);

    m_drivetrain.setDefaultCommand(m_defaultDrive);

    // Configure the button bindings
    configureButtonBindings();
  }

  /*
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    bumper_right.whenPressed(new SwitchGears(m_drivetrain, true));
    bumper_right.whenReleased(new SwitchGears(m_drivetrain, false));

    bumper_left.whenPressed(new TurnCompressor(m_pneumatics, true));
    bumper_left.whenReleased(new TurnCompressor(m_pneumatics, false));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_auto;
  }
}
