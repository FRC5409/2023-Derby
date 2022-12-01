package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class CompressorOnOff extends CommandBase {

    private final Pneumatics m_pneumatics;
    private final boolean compressorOn;

    public CompressorOnOff(Pneumatics pneumatics, boolean on) {
        m_pneumatics = pneumatics;
        compressorOn = on;
        addRequirements(m_pneumatics);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (compressorOn) {
            m_pneumatics.turnOn();
        } else {
            m_pneumatics.turnOff();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_pneumatics.turnOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}
