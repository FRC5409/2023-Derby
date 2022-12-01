package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDriveTrain;
import frc.robot.Constants.kPneumatics;
import com.ctre.phoenix.sensors.CANCoder;



public class Drivetrain extends SubsystemBase {
    private CANSparkMax m_leftFrontMot;
    private CANSparkMax m_leftBackMot;
    private CANSparkMax m_rightFrontMot;
    private CANSparkMax m_rightBackMot;
    
    private CANCoder m_leftEncoder = new CANCoder(kDriveTrain.leftEncoderID);
    private CANCoder m_rightEncoder = new CANCoder(kDriveTrain.rightEncoderID);

    CANCoderConfiguration config = new CANCoderConfiguration();

    //private static final double kCountsPerRevolution = 1440.0;
    //private static final double kWheelDiameterInch = 3;

    private WPI_Pigeon2 m_gyro = new WPI_Pigeon2(kDriveTrain.pigeonCANID);

    private DifferentialDriveOdometry m_odometry;

    //private UsbCamera camera = new edu.wpi.first.cscore.UsbCamera("Cam 0", 0);

    private Field2d m_field = new Field2d();
 
    private DifferentialDrive m_diffDrive;

    private Solenoid gear;

    public Drivetrain() {
        m_leftFrontMot = new CANSparkMax(kDriveTrain.leftFrontMotID, MotorType.kBrushless);
        m_leftBackMot = new CANSparkMax(kDriveTrain.leftBackMotID, MotorType.kBrushless);
        m_rightFrontMot = new CANSparkMax(kDriveTrain.rightFrontMotID, MotorType.kBrushless);
        m_rightBackMot = new CANSparkMax(kDriveTrain.rightBackMotID, MotorType.kBrushless);

        configMotors();

        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

        m_diffDrive = new DifferentialDrive(m_leftFrontMot, m_rightFrontMot);

        gear = new Solenoid(kPneumatics.MODULE, PneumaticsModuleType.CTREPCM, kPneumatics.gearShiftingModuleID);

        calibrateGyro();

        //config encoders
        config.sensorCoefficient = 2 * Math.PI / 4096.0;

        m_leftEncoder.configAllSettings(config);
        m_rightEncoder.configAllSettings(config);
        //

        resetEncoders();
        m_gyro.reset();

        SmartDashboard.putData("Field", m_field);
    }

    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
        SmartDashboard.putNumber("Left Distance: ", getLeftEncoderPosition());
        SmartDashboard.putNumber("Right Distance: ", getRightEncoderPosition());
        SmartDashboard.putNumber("Average Encoder: ", getAverageEncoder());
        
    }

    public void simulationPeriodic() {
        m_odometry.update(m_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    public void arcadeDrive(double m_forwardSpeed, double m_rotationalSpeed) {
        // SmartDashboard.putNumber("Forward Speed: ", m_forwardSpeed);
        // SmartDashboard.putNumber("Rotational Speed: ", m_rotationalSpeed);
        setRampRate(kDriveTrain.rampRate);
        m_diffDrive.arcadeDrive(m_rotationalSpeed, m_forwardSpeed);
    }

    public void configMotors() {
        m_leftBackMot.follow(m_leftFrontMot);//following front motors
        m_rightBackMot.follow(m_rightFrontMot);

        m_rightFrontMot.setInverted(true);//inverting motors
        m_rightBackMot.setInverted(true);

        m_rightFrontMot.setIdleMode(IdleMode.kBrake);//brake mode
        m_rightBackMot.setIdleMode(IdleMode.kBrake);
        m_leftFrontMot.setIdleMode(IdleMode.kBrake);
        m_leftBackMot.setIdleMode(IdleMode.kBrake);

        m_rightFrontMot.setSmartCurrentLimit(kDriveTrain.motorCurrentLimit);//current limiting
        m_rightBackMot.setSmartCurrentLimit(kDriveTrain.motorCurrentLimit);
        m_leftFrontMot.setSmartCurrentLimit(kDriveTrain.motorCurrentLimit);
        m_leftBackMot.setSmartCurrentLimit(kDriveTrain.motorCurrentLimit);
    }

    public void setRampRate(double rampRate) {
        m_leftFrontMot.setClosedLoopRampRate(rampRate);//settings close loop ramp rate
        m_rightFrontMot.setClosedLoopRampRate(rampRate);
    }

    
    public Double getLeftEncoderPosition() {
        return m_leftEncoder.getPosition() * -1; //interverting because backwards
    }

    public Double getRightEncoderPosition() {
        return m_rightEncoder.getPosition();
    }

    public Double getAverageEncoder() {
        return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2;//getting average distance onf encoders
    }
    
    public Double getLeftEncoderVelo() {
        return m_leftEncoder.getVelocity();
    }

    public Double getRightEncoderVelo() {
        return m_rightEncoder.getVelocity();
    }

    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    public void setGear(boolean fast) {
        gear.set(fast);
    }

    public void switchGears() {
        gear.toggle();
    }

    public void resetGyro() {
        m_gyro.reset();
    }

    public Double getGyroRotation() {
        return m_gyro.getAngle();
    }

    public Double getGyroHead() {
        return m_gyro.getRotation2d().getDegrees();
    }

    public Double getGyroRate() {
        return m_gyro.getRate();
    }

    public void calibrateGyro() {
        m_gyro.calibrate();
    }

    public boolean getSolenoidValue() {
        return gear.get();
    }
}