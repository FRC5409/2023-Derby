package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDriveTrain;
import frc.robot.commands.Drivefwd;

// Drivetrain subsystem
public class DriveTrain extends SubsystemBase {
    private final CANSparkMax m_lFrontDrive_C14;
    private final CANSparkMax m_rFrontDrive_C15;
    private final CANSparkMax m_lRearDrive_C4;
    private final CANSparkMax m_rRearDrive_C6;

    private final DifferentialDrive m_drive;
    private int m_driveMode = kDriveTrain.AADL_DRIVE;
    public void tankDrive(int i, int j) {
    }
    public void tankDrive(float driveSpeed, float driveSpeed2) {
    }

    // Drivetrain constructor
    public DriveTrain() {
        m_lFrontDrive_C14 = new CANSparkMax(Constants.kDriveTrain.kLeftDriveFront, MotorType.kBrushless);
    
        m_lFrontDrive_C14.restoreFactoryDefaults();
        m_lFrontDrive_C14.setIdleMode(IdleMode.kBrake);
        m_lFrontDrive_C14.setSmartCurrentLimit(60);
        m_lFrontDrive_C14.setInverted(true);
        m_lFrontDrive_C14.burnFlash();
    }
}
