package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kAuto;
import frc.robot.subsystems.Drivetrain;

public class Auto extends SequentialCommandGroup {

    public Auto(Drivetrain drivetrain) {
        for (int i = 0; i < kAuto.autoDistance.length; i++) {
            
            addCommands(new AutoDriveStraight(drivetrain, kAuto.autoSpeed, kAuto.autoDistance[i])
            ,new AutoTurn(drivetrain, kAuto.autoTurnSpeed, kAuto.autoTurning[i])
            );
        }
    }
}