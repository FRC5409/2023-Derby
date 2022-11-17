//RANDOM, IGNORABLE NUMBERS

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class kDriveTrain {   
    // Motor CAN IDs
    public static final int kLeftDriveFront = 14;
    public static final int kLeftDriveRear = 4;
    
    public static final int kRightDriveRear = 6;
    public static final int kRightDriveFront = 15;

    // Drive mode constants
    public static final int ARCADE_DRIVE = 1; // Arcade Drive
    public static final int AADL_DRIVE = 2; // Addl Drive (Arcade drive)
    public static final int C_DRIVE = 3; // Curvature drive
    public static final int T_DRIVE = 4; // Tank Drive

    public static final double UP_SHIFT = 300;
    public static final double DOWN_SHIFT = 260;

    public static final int MOTOR_CURRENT_LIMIT = 60;


    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double PDriveVel = 8.5;


    public static final double TrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics DriveKinematics = new DifferentialDriveKinematics(
            TrackwidthMeters);

    public static final double MaxSpeedMetersPerSecond = 3;
    public static final double MaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double RamseteB = 2;
    public static final double RamseteZeta = 0.7;
    public static final int CANPigeon = 0;
}

public static class kPneumatics {
    public static final int PCMId = 0;
    public static final int MODULE = 0;
}
  public static final class DriveConstants {
    
    public static final int kLeftMotor1Port = 14; //front
    public static final int kLeftMotor2Port = 4;
    public static final int kRightMotor1Port = 15;
    public static final int kRightMotor2Port = 6;

    public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRightEncoderPorts = new int[] {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final boolean kGyroReversed = true;


    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // These two values are "angular" kV and kA
    public static final double kvVoltSecondsPerRadian = 1.5;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
        LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter,
            kvVoltSecondsPerRadian,
            kaVoltSecondsSquaredPerRadian);

    // Example values only -- use what's on your physical robot!
    public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
    public static final double kDriveGearing = 8;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
