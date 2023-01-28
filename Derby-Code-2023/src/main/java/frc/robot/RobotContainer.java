// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultDrive;
import frc.robot.commands.GearShift;
import frc.robot.commands.LimeLightCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Limelight;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //controller
  private final CommandXboxController sys_joystickMain = new CommandXboxController(0);
  private final CommandXboxController sys_joystickSecondary = new CommandXboxController(1);
  private final ArrayList<CommandXboxController> sys_joysticks = new ArrayList<CommandXboxController>();
  
  // Subsystems
  private final Drivetrain sys_drivetrain = new Drivetrain();
  private final Pneumatics sys_pneumatics = new Pneumatics();
  private final Limelight sys_Limelight = new Limelight();

  //commands
  private final DefaultDrive cmd_defaultDrive = new DefaultDrive(sys_drivetrain, sys_joysticks);
  private final GearShift cmd_fastGear = new GearShift(sys_drivetrain, true);
  private final GearShift cmd_slowGear = new GearShift(sys_drivetrain, false);
  private final LimeLightCommand cmd_limeLight = new LimeLightCommand(sys_Limelight);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    sys_drivetrain.setDefaultCommand(cmd_defaultDrive);
    
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    sys_joystickMain.leftBumper().onTrue(cmd_fastGear).onFalse(cmd_slowGear);
    sys_joystickSecondary.start().onTrue(Commands.runOnce(() -> sys_drivetrain.changeJoystick()));

    sys_joystickMain.povRight().onTrue(cmd_limeLight);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   // return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
