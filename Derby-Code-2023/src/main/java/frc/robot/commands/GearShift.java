package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GearShift extends CommandBase {

    private final Drivetrain m_drivetrain;
    private final boolean fast;

    public GearShift(Drivetrain drivetrain, boolean fast) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drivetrain = drivetrain;
        this.fast = fast;


        addRequirements(m_drivetrain);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println(fast);
        m_drivetrain.switchGear(fast);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}
