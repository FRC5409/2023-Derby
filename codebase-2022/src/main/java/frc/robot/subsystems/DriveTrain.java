// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kDriveTrain;

/**
 * DriveTrain subsystem
 */
public class DriveTrain extends SubsystemBase {
    private final CANSparkMax mot_leftDriveFront_sparkmax_C14;
    private final CANSparkMax mot_rightDriveFront_sparkmax_C15;
    private final CANSparkMax mot_leftDriveRear_sparkmax_C4;
    private final CANSparkMax mot_rightDriveRear_sparkmax_C6;

    private final WPI_CANCoder enc_leftCANCoder;
    private final WPI_CANCoder enc_rightCANCoder;
    private final CANCoderConfiguration enc_config;

    private final DifferentialDrive m_drive;
    private int m_driveMode = kDriveTrain.AADL_DRIVE;

    private final Solenoid ssl_gear;
    private boolean m_allowShift = false;
    private long m_timeSinceShift = 0;

    private final WPI_PigeonIMU m_gyro;
    private final DifferentialDriveOdometry m_odometry;

    /** Creates a new DriveTrain. */
    public DriveTrain() {
        mot_leftDriveFront_sparkmax_C14 = new CANSparkMax(Constants.kDriveTrain.kLeftDriveFront, MotorType.kBrushless);
    
        mot_leftDriveFront_sparkmax_C14.restoreFactoryDefaults();
        mot_leftDriveFront_sparkmax_C14.setIdleMode(IdleMode.kBrake);
        mot_leftDriveFront_sparkmax_C14.setSmartCurrentLimit(60);
        mot_leftDriveFront_sparkmax_C14.setInverted(true);
        mot_leftDriveFront_sparkmax_C14.burnFlash();
    
        mot_leftDriveRear_sparkmax_C4 = new CANSparkMax(Constants.kDriveTrain.kLeftDriveRear, MotorType.kBrushless);
        mot_leftDriveRear_sparkmax_C4.restoreFactoryDefaults();
        mot_leftDriveRear_sparkmax_C4.setIdleMode(IdleMode.kBrake);
        mot_leftDriveRear_sparkmax_C4.setSmartCurrentLimit(60);
        mot_leftDriveRear_sparkmax_C4.setInverted(true);
        
        mot_leftDriveFront_sparkmax_C14.follow(ExternalFollower.kFollowerDisabled, 0);
    
        mot_leftDriveRear_sparkmax_C4.follow(mot_leftDriveFront_sparkmax_C14);
        mot_leftDriveRear_sparkmax_C4.burnFlash();
    
        mot_rightDriveFront_sparkmax_C15 = new CANSparkMax(Constants.kDriveTrain.kRightDriveFront, MotorType.kBrushless);
        mot_rightDriveFront_sparkmax_C15.restoreFactoryDefaults();
        mot_rightDriveFront_sparkmax_C15.setIdleMode(IdleMode.kBrake);
        mot_rightDriveFront_sparkmax_C15.setSmartCurrentLimit(60);
        mot_rightDriveFront_sparkmax_C15.burnFlash();
        mot_rightDriveFront_sparkmax_C15.setInverted(false);
    
        mot_rightDriveRear_sparkmax_C6 = new CANSparkMax(Constants.kDriveTrain.kRightDriveRear, MotorType.kBrushless);
        mot_rightDriveRear_sparkmax_C6.restoreFactoryDefaults();
        mot_rightDriveRear_sparkmax_C6.setIdleMode(IdleMode.kBrake);
        mot_rightDriveRear_sparkmax_C6.setSmartCurrentLimit(60);
        
        mot_rightDriveFront_sparkmax_C15.follow(ExternalFollower.kFollowerDisabled, 0);
    
        mot_rightDriveRear_sparkmax_C6.follow(mot_rightDriveFront_sparkmax_C15);
        mot_rightDriveRear_sparkmax_C6.burnFlash();

        // CANCoders
        enc_leftCANCoder = new WPI_CANCoder(Constants.kDriveTrain.kCANCoders.kLeftCANCoder);
        enc_rightCANCoder = new WPI_CANCoder(Constants.kDriveTrain.kCANCoders.kRightCANCoder);

        resetCANCoders();

        // Config CANCoders
        enc_config = new CANCoderConfiguration();
        enc_config.sensorCoefficient = Constants.kDriveTrain.kCANCoders.kSensorCoefficient;
        enc_config.unitString = Constants.kDriveTrain.kCANCoders.kUnitString;
        enc_config.sensorTimeBase = SensorTimeBase.PerSecond;
        enc_leftCANCoder.configAllSettings(enc_config);
        enc_rightCANCoder.configAllSettings(enc_config);

        /**
         * ------------------ DIFFERENTIAL DRIVE ------------------
         * Decleration: 
         */
        m_drive = new DifferentialDrive(mot_leftDriveFront_sparkmax_C14, mot_rightDriveFront_sparkmax_C15);

        ssl_gear = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

        SmartDashboard.putData(ssl_gear);

        m_gyro = new WPI_PigeonIMU(Constants.kGyro.kID);
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    }

    /**
     * This method is called once per scheuler run and is used to update smart
     * dashboard data.
     */
    public void periodic() {

        displayDriveModeData();

        SmartDashboard.putString("Drive Mode", getDriveModeName());


        // CANCoders
        SmartDashboard.putNumber("CANCoder L Vel", getLeftCANCoderVelocity());
        SmartDashboard.putNumber("CANCoder R Vel", getRightCANCoderVelocity());
        SmartDashboard.putNumber("CANCoder L Dist", getLeftDistance());
        SmartDashboard.putNumber("CANCoder R Dist", getRightDistance());

        m_odometry.update(m_gyro.getRotation2d(), getLeftDistance(), getRightDistance());
    }

    @Override
    public void simulationPeriodic() {

    }

    /**
     * This method sets the drive mode to the next one
     */
    public void nextDriveMode() {
        m_driveMode++;

        if (m_driveMode > 4)
            m_driveMode = kDriveTrain.ARCADE_DRIVE;
    }

    /**
     * This method sets the drive mode to the previous one
     */
    public void previousDriveMode() {
        m_driveMode--;

        if (m_driveMode < 1)
            m_driveMode = kDriveTrain.T_DRIVE;
    }

    /**
     * This method returns the current drive mode
     * 
     * @return int value corresponding to the current drive mode
     */
    public int getDriveMode() {
        return m_driveMode;
    }

    /**
     * This method will return the string value corresponding to the current drive
     * mode. Ideally should be used as a way to display the data to the driver
     * station.
     * 
     * @return String value for current drive mode
     */
    public String getDriveModeName() {
        switch (m_driveMode) {
            case kDriveTrain.ARCADE_DRIVE:
                return "ARCADE DRIVE";
            case kDriveTrain.AADL_DRIVE:
                return "AADL DRIVE";
            case kDriveTrain.C_DRIVE:
                return "CURVATURE DRIVE";
            case kDriveTrain.T_DRIVE:
                return "TANK DRIVE";
            default:
                return "ARCADE DRIVE";
        }
    }

    /**
     * This method will display values to the smart dashboard that relate to the
     * current drive mode.
     */
    public void displayDriveModeData() {
        switch (m_driveMode) {
            case kDriveTrain.ARCADE_DRIVE:
                SmartDashboard.delete("ADT_Acceleration");

                SmartDashboard.delete("CD_Speed");
                SmartDashboard.delete("CD_Turn");
                SmartDashboard.delete("CD_Quik Turn");

                SmartDashboard.delete("TD_Left Speed");
                SmartDashboard.delete("TD_Right Speed");

            case kDriveTrain.AADL_DRIVE:
                SmartDashboard.delete("ADS_Acceleration");

                SmartDashboard.delete("CD_Speed");
                SmartDashboard.delete("CD_Turn");
                SmartDashboard.delete("CD_Quik Turn");

                SmartDashboard.delete("TD_Left Speed");
                SmartDashboard.delete("TD_Right Speed");
                break;
            case kDriveTrain.C_DRIVE:
                SmartDashboard.delete("ADS_Acceleration");

                SmartDashboard.delete("ADT_Acceleration");

                SmartDashboard.delete("TD_Left Speed");
                SmartDashboard.delete("TD_Right Speed");
                break;
            case kDriveTrain.T_DRIVE:
                SmartDashboard.delete("ADS_Acceleration");
                SmartDashboard.delete("ADT_Acceleration");

                SmartDashboard.delete("CD_Speed");
                SmartDashboard.delete("CD_Turn");
                SmartDashboard.delete("CD_Quik Turn");
                break;
        }
    }

    /**
     * This method will move the robot in a direction based on the given parameters
     * 
     * @param acceleration Motor speed (-1.0 - 1.0)
     * @param turn         Motor horizontal speed (-1.0 - 1.0)
     */
    public void arcadeDrive(final double acceleration, final double turn) {
        m_drive.arcadeDrive(acceleration, turn, true);
        SmartDashboard.putNumber("ADS_Acceleration", acceleration);
    }

    /**
     * This method will rotate the motors with the given parameters
     * 
     * @param acceleration Right acceleration
     * @param deceleration Left accelertation
     * @param turn         Turn percentage
     */
    public void aadlDrive(final double acceleration, final double deceleration, final double turn) {
        double accelrate = acceleration - deceleration;

        m_drive.arcadeDrive(accelrate, turn, true);
        SmartDashboard.putNumber("ADT_Acceleration", accelrate);
    }

    /**
     * This method will move the robot in a direction based on the given parameters
     * 
     * @param speed     Motor straight speed (-1.0 - 1.0)
     * @param turn      Motor horizontal speed (-1.0 - 1.0)
     * @param quickTurn Boolean fast turn value
     */
    public void curvDrive(final double speed, final double turn, final boolean quickTurn) {
        m_drive.curvatureDrive(speed, turn, quickTurn);
        SmartDashboard.putNumber("CD_Speed", speed);
        SmartDashboard.putNumber("CD_Turn", turn);
        SmartDashboard.putBoolean("CD_Quik Turn", quickTurn);
    }

    /**
     * This method will move the robot in a direction based on the given speeds
     * 
     * @param leftSpeed  Left motor speed
     * @param rightSpeed Right motor speed
     */
    public void tankDrive(final float leftSpeed, final float rightSpeed) {
        m_drive.tankDrive(leftSpeed, rightSpeed);

        SmartDashboard.putNumber("TD_Left Speed", leftSpeed);
        SmartDashboard.putNumber("TD_Right Speed", rightSpeed);
    }

    /**
     * This method will put the robot in high gear
     */
    public void fastShift() {
        ssl_gear.set(true);
    }

    /**
     * This method will put the robot is low gear
     */
    public void slowShift() {
        ssl_gear.set(false);
    }

    public void resetCANCoders() {
        enc_leftCANCoder.setPosition(0);
        enc_rightCANCoder.setPosition(0);
    }

    public double getLeftDistance() {
        return -enc_leftCANCoder.getPosition();
    }

    public double getRightDistance() {
        return enc_rightCANCoder.getPosition();
    }

    public double getLeftCANCoderVelocity() {
        return -enc_leftCANCoder.getVelocity();
    }

    public double getRightCANCoderVelocity() {
        return enc_rightCANCoder.getVelocity();
    }

    public Pose2d getPose2d() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftCANCoderVelocity(), getRightCANCoderVelocity());
    }

    public void resetOdometry(Pose2d pose) {
        resetCANCoders();
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public double getTurnRate() {
        return -m_gyro.getRate();
    }

    public void tankDriveVoltages(double lVolts, double rVolts) {
        mot_leftDriveFront_sparkmax_C14.setVoltage(lVolts);
        mot_rightDriveFront_sparkmax_C15.setVoltage(lVolts);
        m_drive.feed();
    }
    
}
