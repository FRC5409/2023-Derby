package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SwitchGears extends CommandBase {
    private final Drivetrain m_drivetrain;
    private final boolean m_speed;

    public SwitchGears(Drivetrain drivetrain, boolean speed) {
        m_drivetrain = drivetrain;
        m_speed = speed;
        addRequirements(m_drivetrain);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drivetrain.setGear(m_speed);
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
