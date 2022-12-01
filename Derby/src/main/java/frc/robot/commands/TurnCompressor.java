package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class TurnCompressor extends CommandBase {

    private Pneumatics m_pneumatics;
    private boolean m_state;

    public TurnCompressor(Pneumatics pneumatics, boolean on) {
        m_pneumatics = pneumatics;
        m_state = on;
        addRequirements(m_pneumatics);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_state) {
            m_pneumatics.turnOn();
        } else {
            m_pneumatics.turnOff();
        }
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
