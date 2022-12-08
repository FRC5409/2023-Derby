// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class kDriveTrain {   
        // Motor CAN IDs
        public static final int kLeftDriveFront = 14;
        public static final int kLeftDriveRear = 4;
        
        public static final int kRightDriveRear = 6;
        public static final int kRightDriveFront = 15;

        public static final double kWheelDiameter = 101.6;

        public static class kCANCoders {
            public static final int kLeftCANCoder = 11;
            public static final int kRightCANCoder = 10;

            public static final double kCountsPerRevolution = 4096;
            public static final double kSensorCoefficient = (Math.PI * kDriveTrain.kWheelDiameter) / kCountsPerRevolution;
            public static final String kUnitString = "mm";
        }

        // Drive mode constants
        public static final int ARCADE_DRIVE = 1; // Arcade Drive
        public static final int AADL_DRIVE = 2; // Addl Drive (Arcade drive)
        public static final int C_DRIVE = 3; // Curvature drive
        public static final int T_DRIVE = 4; // Tank Drive

        public static final double UP_SHIFT = 300;
        public static final double DOWN_SHIFT = 260;

        public static final int MOTOR_CURRENT_LIMIT = 60;

        public static class Characterization {
            public static final double ksVolts = 0.13874;
            public static final double kvVoltSecondsPerMeter = 5.5055;
            public static final double kaVoltSecondsSquaredPerMeter = 0.34755;

            public static final double kPDriveVel = 5.6098;

            public static final double kTrackwidthMeters = 0.5;
            public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

            public static final double kMaxSpeedMetersPerSecond = 3;
            public static final double kMaxAccelerationMetersPerSecondSquared = 3;

            public static final double kRamseteB = 2;
            public static final double kRamseteZeta = 0.7;
        }
    }

    public static class kPneumatics {
        public static final int PCMId = 0;
        public static final int MODULE = 0;
    }

    public static class kGyro {
        public static final int kID = 23;
    }
}
