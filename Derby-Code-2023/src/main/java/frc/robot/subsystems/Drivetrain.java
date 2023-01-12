package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDrivetrain;

public class Drivetrain extends SubsystemBase {

    private CANSparkMax mot_leftFront;
    private CANSparkMax mot_leftRear;
    private CANSparkMax mot_rightFront;
    private CANSparkMax mot_rightRear;

    private final DifferentialDrive m_drive;

    public Drivetrain() {
        mot_leftFront = new CANSparkMax(kDrivetrain.kMotors.leftFrontId, MotorType.kBrushless);
        mot_leftRear = new CANSparkMax(kDrivetrain.kMotors.leftRearId, MotorType.kBrushless);
        mot_rightFront = new CANSparkMax(kDrivetrain.kMotors.rightFrontId, MotorType.kBrushless);
        mot_rightRear = new CANSparkMax(kDrivetrain.kMotors.rightRearId, MotorType.kBrushless);

        mot_leftFront.restoreFactoryDefaults();
        mot_leftRear.restoreFactoryDefaults();
        mot_rightFront.restoreFactoryDefaults();
        mot_rightRear.restoreFactoryDefaults();

        mot_rightFront.setInverted(true);
        mot_rightRear.setInverted(true);

        mot_leftFront.setSmartCurrentLimit(kDrivetrain.currentLimit);
        mot_leftRear.setSmartCurrentLimit(kDrivetrain.currentLimit);
        mot_rightFront.setSmartCurrentLimit(kDrivetrain.currentLimit);
        mot_rightRear.setSmartCurrentLimit(kDrivetrain.currentLimit);

        rampRate(kDrivetrain.rampRate);

        mot_leftRear.follow(mot_leftFront);
        mot_rightRear.follow(mot_rightFront);

        m_drive = new DifferentialDrive(mot_leftFront, mot_rightFront);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Left Front Motor Temp: ", mot_leftFront.getMotorTemperature());
        SmartDashboard.putNumber("Left Rear Motor Temp: ", mot_leftRear.getMotorTemperature());
        SmartDashboard.putNumber("Right Front Motor Temp: ", mot_rightFront.getMotorTemperature());
        SmartDashboard.putNumber("Right Rear Motor Temp: ", mot_rightRear.getMotorTemperature());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

    public void rampRate(double ramp) {
        mot_leftFront.setOpenLoopRampRate(ramp);
        mot_leftRear.setOpenLoopRampRate(ramp);
        mot_rightFront.setOpenLoopRampRate(ramp);
        mot_rightRear.setOpenLoopRampRate(ramp);
    }

    public void defaultDrive(double forwardSpeed, double rotationalSpeed) {
        m_drive.arcadeDrive(forwardSpeed, rotationalSpeed);
    }

}
