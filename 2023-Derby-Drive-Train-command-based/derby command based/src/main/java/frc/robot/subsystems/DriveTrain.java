package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDriveTrain;

// Drivetrain subsystem
public class DriveTrain extends SubsystemBase {
    private final CANSparkMax m_lFrontDrive;
    private final CANSparkMax m_rFrontDrive;
    private final CANSparkMax m_lRearDrive;
    private final CANSparkMax m_rRearDrive;

    private final DifferentialDrive m_drive;
    private int m_driveMode = kDriveTrain.AADL_DRIVE;

    
}
