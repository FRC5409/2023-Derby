// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.kDrivetrain;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightR;

public class ConeNodeAim extends CommandBase {

	private final LimelightR sys_limelightR;
	private final Drivetrain sys_drivetrain;
	private final CommandXboxController m_joystick;

	double forwardSpeed, dir, turning;

	/** Creates a new TargetAim. */
	public ConeNodeAim(LimelightR limelightR, Drivetrain drivetrain, CommandXboxController joystick) {

		sys_limelightR = limelightR;
		sys_drivetrain = drivetrain;
		m_joystick = joystick;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(sys_drivetrain, sys_limelightR);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		sys_limelightR.turnOn();
		// sys_limelight.setData("pipeline", 1);
		// System.out.println("Initialized");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		forwardSpeed = m_joystick.getRightTriggerAxis() - m_joystick.getLeftTriggerAxis();

		if (!sys_limelightR.isVisible()) {dir = sys_limelightR.getTurningDir();} // dir on the controller
		else {dir = sys_limelightR.getXOffset();}
		
		// dir = sys_limelightR.getXOffset() / Math.abs(sys_limelightR.getXOffset());
		// dir returns -1 or 1 depending on if it's positive or if it's negative

		turning = dir * kDrivetrain.kCNodeTargetSpeed + m_joystick.getLeftX(); // Since the X Offset keeps decreasing, the turning speed will decrease

		sys_drivetrain.defaultDrive(forwardSpeed, turning); // Tune this
		// System.out.println("Executed");
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		sys_drivetrain.defaultDrive(0, 0);
		sys_limelightR.turnOff();
		// System.out.println("Ended");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Math.abs(sys_limelightR.getXOffset()) <= Constants.kLimelight.targetStopAngle && sys_limelightR.isVisible();
	}
}
