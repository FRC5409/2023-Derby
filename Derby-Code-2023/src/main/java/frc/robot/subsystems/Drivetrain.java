package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kPneumatics;

public class Drivetrain extends SubsystemBase {

    private final CANSparkMax mot_leftFront;
    private final CANSparkMax mot_leftRear;
    private final CANSparkMax mot_rightFront;
    private final CANSparkMax mot_rightRear;

    private final DifferentialDrive m_drive;

    private final Solenoid gearShift;

    private int currentJoystick = 0;

    public Drivetrain() {
        mot_leftFront = new CANSparkMax(kDrivetrain.kMotors.leftFrontId, MotorType.kBrushless);
        mot_leftRear = new CANSparkMax(kDrivetrain.kMotors.leftRearId, MotorType.kBrushless);
        mot_rightFront = new CANSparkMax(kDrivetrain.kMotors.rightFrontId, MotorType.kBrushless);
        mot_rightRear = new CANSparkMax(kDrivetrain.kMotors.rightRearId, MotorType.kBrushless);

        mot_leftFront.restoreFactoryDefaults();
        mot_leftRear.restoreFactoryDefaults();
        mot_rightFront.restoreFactoryDefaults();
        mot_rightRear.restoreFactoryDefaults();

        mot_rightRear.follow(mot_leftFront);
        mot_leftRear.follow(mot_rightFront);

        mot_rightFront.setInverted(true);
        mot_rightRear.setInverted(true);

        mot_leftFront.setIdleMode(IdleMode.kBrake);
        mot_rightFront.setIdleMode(IdleMode.kBrake);
        mot_leftRear.setIdleMode(IdleMode.kBrake);
        mot_rightRear.setIdleMode(IdleMode.kBrake);


        mot_leftFront.setSmartCurrentLimit(kDrivetrain.currentLimit);
        mot_leftRear.setSmartCurrentLimit(kDrivetrain.currentLimit);
        mot_rightFront.setSmartCurrentLimit(kDrivetrain.currentLimit);
        mot_rightRear.setSmartCurrentLimit(kDrivetrain.currentLimit);

        // rampRate(kDrivetrain.rampRate);

        mot_leftFront.burnFlash();
        mot_leftRear.burnFlash();
        mot_rightFront.burnFlash();
        mot_rightRear.burnFlash();

        m_drive = new DifferentialDrive(mot_leftFront, mot_rightFront);

        gearShift = new Solenoid(kPneumatics.pneumaticsModule, kPneumatics.gearShiftPCMId);
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

    public void switchGear(boolean fast) {
        System.out.println("Speed: " + fast);
        gearShift.set(fast);
        // gearShift.toggle();

    }

    public int getCurrentJoystick() {
        return currentJoystick;
    }

    public void changeJoystick() {
        currentJoystick = (currentJoystick + 1) % 2;
    }

}
