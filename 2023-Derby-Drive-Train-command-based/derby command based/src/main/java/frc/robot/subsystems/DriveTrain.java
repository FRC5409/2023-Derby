package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Drivetrain subsystem
public class DriveTrain extends SubsystemBase {
    private final CANSparkMax m_lFrontDrive_C14;
    private final CANSparkMax m_rFrontDrive_C15;
    private final CANSparkMax m_lRearDrive_C4;
    private final CANSparkMax m_rRearDrive_C6;

    // Instantiate DifferentialDrive
    private final DifferentialDrive m_diffDrive;

    // Drivetrain constructor
    public DriveTrain() {

        // Set values for motors
        m_lFrontDrive_C14 = new CANSparkMax(Constants.kDriveTrain.CANLeftDriveFront, MotorType.kBrushless);
        m_lFrontDrive_C14.restoreFactoryDefaults();
        m_lFrontDrive_C14.setIdleMode(IdleMode.kBrake);
        m_lFrontDrive_C14.setSmartCurrentLimit(60);
        m_lFrontDrive_C14.setInverted(true);

        m_lRearDrive_C4 = new CANSparkMax(Constants.kDriveTrain.CANLeftDriveRear, MotorType.kBrushless);
        m_lRearDrive_C4.restoreFactoryDefaults();
        m_lRearDrive_C4.setIdleMode(IdleMode.kBrake);
        m_lRearDrive_C4.setSmartCurrentLimit(60);
        m_lRearDrive_C4.setInverted(true);

        m_rFrontDrive_C15 = new CANSparkMax(Constants.kDriveTrain.CANRightDriveFront, MotorType.kBrushless);
        m_rFrontDrive_C15.restoreFactoryDefaults();
        m_rFrontDrive_C15.setIdleMode(IdleMode.kBrake);
        m_rFrontDrive_C15.setSmartCurrentLimit(60);
        m_rFrontDrive_C15.setInverted(false);

        m_rRearDrive_C6 = new CANSparkMax(Constants.kDriveTrain.CANRightDriveRear, MotorType.kBrushless);
        m_rRearDrive_C6.restoreFactoryDefaults();
        m_rRearDrive_C6.setIdleMode(IdleMode.kBrake);
        m_rRearDrive_C6.setSmartCurrentLimit(60);
        m_rRearDrive_C6.setInverted(false);

        // Set followers
        m_lFrontDrive_C14.follow(ExternalFollower.kFollowerDisabled, 0);
        m_lRearDrive_C4.follow(m_lFrontDrive_C14);

        m_rFrontDrive_C15.follow(ExternalFollower.kFollowerDisabled, 0);
        m_rRearDrive_C6.follow(m_rFrontDrive_C15);

        // Burn flash
        m_lFrontDrive_C14.burnFlash();
        m_lRearDrive_C4.burnFlash();
        m_rFrontDrive_C15.burnFlash();
        m_rRearDrive_C6.burnFlash();

        // Initialize DifferentialDrive
        m_diffDrive = new DifferentialDrive(m_lFrontDrive_C14, m_rFrontDrive_C15);
    }

    public void tankDrive(float lSpeed, float rSpeed) {
        m_diffDrive.tankDrive(lSpeed, rSpeed);
    }

    public void aadlDrive(float fAccel, float rAccel, float turnVal) {
        m_diffDrive.arcadeDrive(fAccel - rAccel, turnVal);
    }
}