// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Controls {
        
        public static final double stickDeadband = 0.075;
        public static final int translationAxis = 1;
        public static final int rotationAxis = 2;
        public static double slewRateLimit = 3;

    }

    public static final class Drive {

        public static final boolean invertGyro = true;

        public static final int frontLeftCAN = 0;
        public static final int frontRightCAN = 1;
        public static final int backLeftCAN = 2;
        public static final int backRightCAN = 3;

        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        public static final double kTrackWidth = Units.inchesToMeters(24);
        public static final double kWheelRadius = Units.inchesToMeters(3);
        public static final double kEncoderResolution = 42;

        public static final double EncoderConversionFactor = 2 * Math.PI * kWheelRadius / kEncoderResolution;

        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0;
        public static final double kV = 0;

        

    }

}
