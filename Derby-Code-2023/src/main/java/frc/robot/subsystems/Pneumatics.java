package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kPneumatics;

public class Pneumatics extends SubsystemBase {

    private final Compressor m_compressor;

    public Pneumatics() {
        m_compressor = new Compressor(kPneumatics.compressorCANID, kPneumatics.pneumaticsModule);
        enableCompressor(); 
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putNumber("Compressor Pressure: ", getCompressorPressure());
        SmartDashboard.putBoolean("Is Enabled", m_compressor.isEnabled());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

    public void enableCompressor() {
        // m_compressor.enableAnalog(kPneumatics.minPressure, kPneumatics.maxPressure);
        System.out.println("Enabled");
        m_compressor.enableDigital();
    }

    public void disableCompressor() {
        m_compressor.disable();
    }

    // public double getCompressorPressure() {
    //     return m_compressor.getPressure();
    // }

}
