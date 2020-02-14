package frc.robot.utilities;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.util.Units;

/**
 * Defines enums for all robot preferences.
 */
public class NRGPreferences {

    private static final LinkedList<BasePrefs> allPrefValues = new LinkedList<BasePrefs>();

    abstract static class BasePrefs
    {
        protected String key;
        protected Object defaultValue;


        protected BasePrefs(String key, Object defaultValue) {
            this.key = key;
            this.defaultValue = defaultValue;
            allPrefValues.add(this);
        }

        public String getKey() {
            return key;
        }

        abstract void writeDefaultValue();

        abstract boolean isDefault();

        // Although not explicitly abstract, each derived class MUST
        // implement getValue(), returning the appropriate data type,
        // and also toString() with consistent formatting. 

        void printIfNotDefault()
        {
            if (!isDefault())
                System.out.println(this);
        }
    }

    public static class NumberPrefs extends BasePrefs
    {
        protected NumberPrefs(String key, double defaultValue) {
            super(key, defaultValue);
        }

        public double getValue() {
            return Preferences.getInstance().getDouble(key, (double)defaultValue);
        }

        public void setValue(double newValue) {
            Preferences.getInstance().putDouble(key, newValue);
        }

        void writeDefaultValue() {
            Preferences.getInstance().putDouble(key, (double)defaultValue);
        }

        boolean isDefault() {
            return getValue() == (double)defaultValue;
        }

        public String toString() {
            return this.key + " Value:" + this.getValue();
        }
    }

    public static class BooleanPrefs extends BasePrefs
    {
        protected BooleanPrefs(String key, boolean defaultValue) {
            super(key, defaultValue);
        }

        public boolean getValue() {
            return Preferences.getInstance().getBoolean(key, (boolean)defaultValue);
        }

        public void setValue(boolean newValue) {
            Preferences.getInstance().putBoolean(key, newValue);
        }

        void writeDefaultValue() {
            Preferences.getInstance().putBoolean(key, (boolean)defaultValue);
        }

        boolean isDefault() {
            return getValue() == (boolean)defaultValue;
        }

        public String toString() {
            return this.key + " Value:" + this.getValue();
        }
    }

    public static class StringPrefs extends BasePrefs
    {
        protected StringPrefs(String key, String defaultValue) {
            super(key, defaultValue);
        }

        public String getValue() {
            return Preferences.getInstance().getString(key, (String)defaultValue);
        }

        public void setValue(String newValue) {
            Preferences.getInstance().putString(key, newValue);
        }

        void writeDefaultValue() {
            Preferences.getInstance().putString(key, (String)defaultValue);
        }

        boolean isDefault() {
            return getValue() == (String)defaultValue;
        }

        public String toString() {
            return this.key + " Value:" + this.getValue();
        }
    }

    public static final NumberPrefs DRIVE_P_TERM = new NumberPrefs("DriveP", 1.11);
    public static final NumberPrefs DRIVE_I_TERM = new NumberPrefs("DriveI", 0);
    public static final NumberPrefs DRIVE_D_TERM = new NumberPrefs("DriveD", 0.0285); 

    public static final NumberPrefs TURN_P_TERM = new NumberPrefs("TurnP", 0.081);
    public static final NumberPrefs TURN_I_TERM = new NumberPrefs("TurnI", 0.00016);
    public static final NumberPrefs TURN_D_TERM = new NumberPrefs("TurnD", 0.0072);

    public static final NumberPrefs DISTANCE_DRIVE_P_TERM = new NumberPrefs("DistanceDriveP", 0.03);
    public static final NumberPrefs DISTANCE_DRIVE_I_TERM = new NumberPrefs("DistanceDriveI", 0.0125);
    public static final NumberPrefs DISTANCE_DRIVE_D_TERM = new NumberPrefs("DistanceDriveD", 0.0075);
    public static final NumberPrefs DISTANCE_TOLERANCE = new NumberPrefs("DistanceTolerance", 0.75);
    
    public static final NumberPrefs PATH_KS_TERM = new NumberPrefs("PathKS", 0.98);
    public static final NumberPrefs PATH_KV_TERM = new NumberPrefs("PathKV", 0.543);
    public static final NumberPrefs PATH_KA_TERM = new NumberPrefs("PathKA", 0.00337);

    public static final NumberPrefs TRACK_WIDTH_METERS = new NumberPrefs("TrackWidthMeters", Units.inchesToMeters(25.0));
    public static final NumberPrefs ENCODER_CPR = new NumberPrefs("EncoderCPR", 1050);
    
    public static final NumberPrefs DRIVE_TO_VISION_TAPE_MIN_POWER = new NumberPrefs("VisionMinPower", 0.15);
    public static final NumberPrefs DRIVE_TO_VISION_TAPE_MAX_POWER = new NumberPrefs("VisionMaxPower", 0.65);
    public static final NumberPrefs CAMERA_ANGLE_SKEW = new NumberPrefs("CameraAngleSkew", -1.7);
    public static final NumberPrefs CAMERA_DISTANCE_SCALE = new NumberPrefs("CameraDistanceScale", 1.0);
    public static final NumberPrefs CAMERA_ANGLE_SCALE = new NumberPrefs("CameraAngleScale", 0.8);

    public static final NumberPrefs TURRET_P_TERM = new NumberPrefs("TurretP", 0.081);
    public static final NumberPrefs TURRET_I_TERM = new NumberPrefs("TurretI", 0.00016);
    public static final NumberPrefs TURRET_D_TERM = new NumberPrefs("TurretD", 0.0072);
    public static final NumberPrefs TURRET_MANUAL_MOTOR_POWER = new NumberPrefs("TurretMotorPower", 0.2);

    public static final NumberPrefs HOOD_MANUAL_MOTOR_POWER = new NumberPrefs("HoodMotorPower", 0.35);

    public static final NumberPrefs CLIMBER_REAR_POWER = new NumberPrefs("ClimberRearPower", 0.95);
    public static final NumberPrefs CLIMBER_REAR_MIN_TICKS = new NumberPrefs("ClimberRearMinTicks", 800);
    public static final NumberPrefs CLIMBER_ARM_WHEELS_POWER = new NumberPrefs("ClimberArmWheelsPower", 0.5);

    public static final NumberPrefs DRIVE_STRAIGHT_MAXPOWER = new NumberPrefs("DriveStraightMaxPower", .5);
    public static final NumberPrefs DRIVE_TO_BALL_MAXPOWER = new NumberPrefs("DriveToBallMaxPower", 0.75);
    
    public static final BooleanPrefs WRITE_DEFAULT = new BooleanPrefs("WriteDefault", true); 
    public static final BooleanPrefs USING_PRACTICE_BOT = new BooleanPrefs("UsingPracticeBot", false);
    public static final BooleanPrefs PATHS_SQUARE_INPUTS = new BooleanPrefs("PathsSquareInputs", false); 
    public static final BooleanPrefs TURN_SQUARE_INPUTS = new BooleanPrefs("TurnSquareInputs", false);
    public static final BooleanPrefs TELEOP_SQUARE_INPUTS = new BooleanPrefs("TeleopSquareInputs", true); 
    public static final BooleanPrefs DRIVE_SQUARE_INPUTS = new BooleanPrefs("DriveSquareInputs", false);
    public static final BooleanPrefs DRIVE_USE_XBOX_CONTROL = new BooleanPrefs("DriveUseXboxControl", false);
    
    public static final StringPrefs TEST_PATH_NAME = new StringPrefs("TestPathName", "LEFT_TO_CARGO_FRONT_LEFT_HATCH");
    

    public static void init() {
        if (NRGPreferences.WRITE_DEFAULT.getValue()) {
            System.out.println("WRITING DEFAULT PREFERENCES");
            NRGPreferences.allPrefValues.forEach(p -> p.writeDefaultValue());
            NRGPreferences.WRITE_DEFAULT.setValue(false);
        } else {
            NRGPreferences.allPrefValues.forEach(p -> p.printIfNotDefault());
        }

    }

}
