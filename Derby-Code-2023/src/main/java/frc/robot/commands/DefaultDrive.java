package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase {

    private final Drivetrain m_drivetrain;
    private final CommandXboxController m_joystick;

    public DefaultDrive(Drivetrain drivetrain, CommandXboxController joystick) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drivetrain = drivetrain;
        m_joystick = joystick;
        addRequirements(m_drivetrain);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double forwardSpeed = m_joystick.getRightTriggerAxis() - m_joystick.getLeftTriggerAxis();
        double rotationalSpeed = m_joystick.getLeftX();

        m_drivetrain.defaultDrive(forwardSpeed, rotationalSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}