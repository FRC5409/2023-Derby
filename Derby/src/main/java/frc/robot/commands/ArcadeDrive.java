package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDrive extends CommandBase {
    private final Drivetrain m_drivetrain;
    private final XboxController m_joystick;
    public static double m_forwardSpeed;
    public static double turn;


    public ArcadeDrive(Drivetrain drivetrain, XboxController joystick) {
        m_drivetrain = drivetrain;
        m_joystick = joystick;
        
        addRequirements(drivetrain);
    }

    public void execute() {
        m_forwardSpeed = m_joystick.getRightTriggerAxis() - m_joystick.getLeftTriggerAxis();

        turn = m_joystick.getLeftX();

        m_drivetrain.arcadeDrive(m_forwardSpeed, turn);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
