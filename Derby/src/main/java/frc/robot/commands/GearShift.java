package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pneumatics;

public class GearShift extends CommandBase {

    private final Drivetrain m_drivetrain; 

    private final boolean gearUp;

    private boolean isDone;

    public GearShift(Drivetrain drivetrain, boolean up) {
        m_drivetrain = drivetrain;
        gearUp = up;
        isDone = false;
        
        addRequirements(drivetrain);
        
    }

    public void periodic() {
        isDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (gearUp) {
            isDone = true;
            m_drivetrain.setGear(true);//setting gear to fast
        } else {
            m_drivetrain.setGear(false);//setting gear to slow
            isDone = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        isDone = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {   
        return isDone;
    }

}
