package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kAuto;
import frc.robot.subsystems.Drivetrain;

public class AutoTurn extends CommandBase {

    private final Drivetrain m_drive;
    private final double m_speed;
    private final double m_degrees;

    private double offset = 0;
    private boolean isDone;

    public AutoTurn(Drivetrain drive, double speed, double degrees) {
        m_drive = drive;
        m_speed = speed;
        m_degrees = degrees;
        isDone = false;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drive.arcadeDrive(0, 0);
        m_drive.resetEncoders();
        m_drive.resetGyro();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        offset = Math.abs(m_drive.getGyroRotation()) - Math.abs(m_degrees);
        m_drive.arcadeDrive(0, m_speed * (m_degrees / Math.abs(m_degrees)) * ((offset * -1) / Math.abs(offset * -1)));
        SmartDashboard.putNumber("turn distance: ", m_degrees);
        SmartDashboard.putNumber("Gryo: ", m_drive.getGyroRotation());
        SmartDashboard.putNumber("Momentem offset: ", offset);
        isDone = (Math.abs(m_drive.getGyroRotation()) >= Math.abs(m_degrees) - kAuto.turnOffset / 2  && offset <= kAuto.turnOffset && Math.abs(m_drive.getGyroRate()) <= kAuto.offsetSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }

}