package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveStraight extends CommandBase {

    private final Drivetrain m_drive;
    private final double m_speed;
    private final double m_distance;

    public AutoDriveStraight(Drivetrain drive, double speed, double inches) {
        m_drive = drive;
        m_speed = speed;
        m_distance = inches;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drive.arcadeDrive(0, 0);
        m_drive.resetEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("Speed: ", m_speed);
        SmartDashboard.putNumber("distance: ", m_distance);
        m_drive.arcadeDrive(m_speed, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
        m_drive.resetEncoders();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_drive.getAverageEncoder()) >= Math.abs(m_distance);
    }

}