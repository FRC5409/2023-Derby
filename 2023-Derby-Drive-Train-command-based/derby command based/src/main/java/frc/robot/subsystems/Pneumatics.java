// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Pneumatics extends SubsystemBase {
    private final Compressor gs_compressor;

    public Pneumatics() {
        gs_compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    }
}
