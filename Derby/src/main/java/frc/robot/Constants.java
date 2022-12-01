// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class kDriveTrain {
        public static int leftFrontMotID = 14;
        public static int leftBackMotID = 4;
        public static int rightFrontMotID = 15;
        public static int rightBackMotID = 6;

        public static int leftEncoderID = 11;
        public static int rightEncoderID = 10;

        public static int motorCurrentLimit = 60;

        public static int pigeonCANID = 9;

        public static double rampRate = 0.6;//Time to go from 0 to full throttle in seconds
    }

    public static final class kAuto {
        public static double autoSpeed = 0.5;

        public static double autoTurnSpeed = 0.4;

        public static double[] autoDistance = { 20, 20, 20, 20};

        public static double[] autoTurning = { 90, -90 , 90, -90};

        public static double turnOffset = 10;//x degrees of offset
        public static double offsetSpeed = 10;//x rate of gyro

    }

    public static class kPneumatics {
        public static int PCMId = 12;
        public static int MODULE = 2;//TODO: this value could be wrong
        public static int maxPSI = 110;
        public static int enablePSI = 60;
        public static int gearShiftingModuleID = 0;
    }

}
