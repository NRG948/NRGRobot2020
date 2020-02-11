package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import frc.robot.utilities.NRGPreferences;

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
        public static final int kLeftMotor1Port = 3;
        public static final int kLeftMotor2Port = 4;
        public static final int kRightMotor1Port = 1;
        public static final int kRightMotor2Port = 2;
        public static final int[] kLeftEncoderPorts = new int[] { 0, 1 };
        public static final int[] kRightEncoderPorts = new int[] { 2, 3 };
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        // TODO Measurable Robot Parameters
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
               NRGPreferences.TRACK_WIDTH_METERS.getValue());
        public static final double kWheelDiameterMeters = 0.152;
        public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / 
                (double) NRGPreferences.ENCODER_CPR.getValue();

        public static final double kPDriveVel = 8.5;
    }

    public static final class ShooterConstants {
        public static final int kSpinMotor1Port = 0;
        public static final int kSpinMotor2Port = 1;
        public static final int kSpinEncoderPort = 4;
    }

    // TODO Robot Characterization Values
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class TurretConstants {
        public static final int kTurretMotorPort = 3;
        public static final int[] kTurretEncoderPorts = new int[] { 6, 7 };
        public static final int kHoodMotorPort = 2;

    }

    public static final class ControlPanelConstants {
        public static final int kPanelSpinnerMotorPort = 7;
    }
}
