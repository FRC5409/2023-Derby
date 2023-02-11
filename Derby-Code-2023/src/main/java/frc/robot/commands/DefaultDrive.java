package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase {

    private final Drivetrain m_drivetrain;
    private final CommandXboxController m_joysticks;

    public DefaultDrive(Drivetrain drivetrain, CommandXboxController joysticks) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drivetrain = drivetrain;
        m_joysticks = joysticks;

        addRequirements(m_drivetrain);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //double forwardSpeed = m_joysticks.get(m_drivetrain.getCurrentJoystick()).getRightTriggerAxis() - m_joysticks.get(m_drivetrain.getCurrentJoystick()).getLeftTriggerAxis();
        double forwardSpeed = m_joysticks.getRightTriggerAxis() - m_joysticks.getLeftTriggerAxis();
        double rotationalSpeed = m_joysticks.getLeftX();
        // System.out.println("Rotational speed" + rotationalSpeed);        //double rotationalSpeed = m_joysticks.get(m_drivetrain.getCurrentJoystick()).getLeftX();

        m_drivetrain.defaultDrive(forwardSpeed, rotationalSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}