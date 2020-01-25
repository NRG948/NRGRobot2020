/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

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
    public static final class DriveConstants {
        public static final boolean kGyroReversed = true;
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 2;
        public static final int kLeftMotor3Port = 3;
        public static final int kRightMotor1Port = 4;
        public static final int kRightMotor2Port = 5;
        public static final int kRightMotor3Port = 6;
        public static final int kPanelSpinnerPort = 7;
        public static final int[] kLeftEncoderPorts = new int[] { 2, 3 };
        public static final int[] kRightEncoderPorts = new int[] { 0, 1 };
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        // TODO Measurable Robot Parameters
        public static final double kTrackwidthMeters = Units.inchesToMeters(25.0);
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);
        public static final int kEncoderCPR = 2005;
        public static final double kWheelDiameterMeters = 0.152;
        public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        // TODO Robot Characterization Values
        public static final double ksVolts = 1.73;
        public static final double kvVoltSecondsPerMeter = 2.21;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0428;

        public static final double kPDriveVel = 8.5;
    }

    // TODO Robot Characterization Values
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
