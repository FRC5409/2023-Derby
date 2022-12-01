package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class TurnOnCompressor extends CommandBase {

    private final Pneumatics m_pneumatics;

    public TurnOnCompressor(Pneumatics pneumatics) {
        m_pneumatics = pneumatics;
        
        addRequirements(m_pneumatics);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_pneumatics.turnOn();
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