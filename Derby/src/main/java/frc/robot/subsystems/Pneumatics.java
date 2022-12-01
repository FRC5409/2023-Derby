package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kPneumatics;;

public class Pneumatics extends SubsystemBase {
    Compressor compress = new Compressor(kPneumatics.MODULE, PneumaticsModuleType.CTREPCM);
    boolean enabled;
    double CompressorPSI;

    public Pneumatics() {
        turnOn();
    }

    @Override
    public void periodic() {
        enabled = compress.enabled();
        CompressorPSI = compress.getPressure();
        SmartDashboard.putBoolean("Compressor enabled? ", enabled);
        SmartDashboard.putNumber("PSI: ", CompressorPSI);
    }

    @Override
    public void simulationPeriodic() {}

    public void turnOff() {
        compress.disable();
    }

    public void turnOn() {
        compress.enableDigital();
    }
}