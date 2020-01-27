/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import java.util.stream.Stream;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.util.Units;

/**
 * Defines enums for all robot preferences.
 */
public class NRGPreferences {

    public enum NumberPrefs {
        DRIVE_P_TERM("DriveP", 0.081),
        DRIVE_I_TERM("DriveI", 0.00016),
        DRIVE_D_TERM("DriveD", 0.0072), 

        TURN_P_TERM("TurnP", 0.081),
        TURN_I_TERM("TurnI", 0.00016),
        TURN_D_TERM("TurnD", 0.0072),

        DISTANCE_DRIVE_P_TERM("DistanceDriveP", 0.03),
        DISTANCE_DRIVE_I_TERM("DistanceDriveI", 0.0125),
        DISTANCE_DRIVE_D_TERM("DistanceDriveD", 0.0075),
        DISTANCE_TOLERANCE("DistanceTolerance", 0.75),
        
        PATH_KS_TERM("PathKS", 0.836),
        PATH_KV_TERM("PathKV", 2.42),
        PATH_KA_TERM("PathKA", 0.119),

        TRACK_WIDTH_METERS("TrackWidthMeters", Units.inchesToMeters(25.0)),
        ENCODER_CPR("EncoderPPR", 4096.0 * 34 / 32),
        
        DRIVE_TO_VISION_TAPE_MIN_POWER("VisionMinPower", 0.15),
        DRIVE_TO_VISION_TAPE_MAX_POWER("VisionMaxPower", 0.65),
        CAMERA_ANGLE_SKEW("CameraAngleSkew", -1.7),
        CAMERA_DISTANCE_SCALE("CameraDistanceScale", 1.0),
        CAMERA_ANGLE_SCALE("CameraAngleScale", 0.8),

        TURRET_P_TERM("TurretP", 0.081),
        TURRET_I_TERM("TurretI", 0.00016),
        TURRET_D_TERM("TurretD", 0.0072),

        CLIMBER_REAR_POWER("ClimberRearPower", 0.95),
        CLIMBER_REAR_MIN_TICKS("ClimberRearMinTicks", 800),
        CLIMBER_ARM_WHEELS_POWER("ClimberArmWheelsPower", 0.5),

        DRIVE_STRAIGHT_MAXPOWER("DriveStraightMaxPower", .5);

        private String key;
        private double defaultValue;

        NumberPrefs(String key, double defaultValue) {
            this.key = key;
            this.defaultValue = defaultValue;
        }

        public String getKey() {
            return key;
        }

        public double getValue() {
            return Preferences.getInstance().getDouble(key, defaultValue);
        }

        void writeDefaultValue() {
            Preferences.getInstance().putDouble(key, defaultValue);
        }

        public String toString() {
            return this.key + " Value:" + this.getValue();
        }
    }

    public enum BooleanPrefs {

        WRITE_DEFAULT("WriteDefault", true), USING_PRACTICE_BOT("UsingPracticeBot", false),
        PATHS_SQUARE_INPUTS("PathsSquareInputs", false), TURN_SQUARE_INPUTS("TurnSquareInputs", false),
        TELEOP_SQUARE_INPUTS("TeleopSquareInputs", true), DRIVE_SQUARE_INPUTS("DriveSquareInputs", false);

        private String key;
        private boolean defaultValue;

        BooleanPrefs(String key, boolean defaultValue) {
            this.key = key;
            this.defaultValue = defaultValue;
        }

        public String getKey() {
            return key;
        }

        public boolean getValue() {
            return Preferences.getInstance().getBoolean(key, defaultValue);
        }

        public void setValue(boolean newValue) {
            Preferences.getInstance().putBoolean(key, newValue);
        }

        void writeDefaultValue() {
            Preferences.getInstance().putBoolean(key, defaultValue);
        }

        public String toString() {
            return this.key + " Value:" + this.getValue();
        }
    }

    public enum StringPrefs {

        TEST_PATH_NAME("TestPathName", "LEFT_TO_CARGO_FRONT_LEFT_HATCH");

        private String key;
        private String defaultValue;

        StringPrefs(String key, String defaultValue) {
            this.key = key;
            this.defaultValue = defaultValue;
        }

        public String getKey() {
            return key;
        }

        public String getValue() {
            return Preferences.getInstance().getString(key, defaultValue);
        }

        void writeDefaultValue() {
            Preferences.getInstance().putString(key, defaultValue);
        }

        public String toString() {
            return this.key + " Value:" + this.getValue();
        }
    }

    public static void init() {
        if (BooleanPrefs.WRITE_DEFAULT.getValue()) {
            System.out.println("WRITING DEFAULT PREFERENCES");
            Stream.of(NumberPrefs.values()).forEach(p -> p.writeDefaultValue());
            Stream.of(BooleanPrefs.values()).forEach(p -> p.writeDefaultValue());
            Stream.of(StringPrefs.values()).forEach(p -> p.writeDefaultValue());
            BooleanPrefs.WRITE_DEFAULT.setValue(false);
        } else {
            Stream.of(NumberPrefs.values()).filter(p -> p.getValue() != p.defaultValue).forEach(System.out::println);
            Stream.of(BooleanPrefs.values()).filter(p -> p.getValue() != p.defaultValue).forEach(System.out::println);
            Stream.of(StringPrefs.values()).filter(p -> p.getValue() != p.defaultValue).forEach(System.out::println);
        }

    }

}
