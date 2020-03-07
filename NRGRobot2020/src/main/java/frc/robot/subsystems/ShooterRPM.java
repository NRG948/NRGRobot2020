package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.Average;
import frc.robot.utilities.MathUtil;
import frc.robot.utilities.NRGPreferences;

/**
 * Robot Subsystem that controls the rotation rate of the shooter flywheel.
 * 
 * The rotation rate in revolutions-per-minute (RPM) is measured using a standard
 * unidirectional encoder (i.e. a {link @Counter}).
 * 
 * Feedback control is accomplished using the Take-Back-Half algorithm. Refer to:
 * https://www.vexwiki.org/programming/controls_algorithms/tbh
 * https://www.chiefdelphi.com/t/paper-take-back-half-shooter-wheel-speed-control/121640/10
 * for more details.
 */
public class ShooterRPM extends SubsystemBase {

  private static final double GAIN = 0.0008;
  private static final double MIN_RPM = 0;
  private static final double MAX_RPM = 5100; // figure out actual max rpm
  private static final double TICKS_PER_FLYWHEEL_REVOLUTION = 645;
  private static final double NANOSECS_PER_SEC = 1000 * 1000 * 1000;
  private static final double NANOSECS_PER_MINUTE = 60 * NANOSECS_PER_SEC;
  
  private final Victor spinMotor1 = new Victor(ShooterConstants.kSpinMotor1Port);
  private final Victor spinMotor2 = new Victor(ShooterConstants.kSpinMotor2Port);
  private final Counter spinMotorEncoder = new Counter(ShooterConstants.kSpinEncoderPort);
  private final LimelightVision limelightVision;

  private double goalRPM = 0;  // The desired RPM for the shooter wheels
  private double error = 0;  // The most recent RPM error
  private double previousError = 0;  // The RPM error from the previous feedback cycle
  private double tbh = 0; // Take-back-half value
  private double motorPower = 0;
  private double lastMotorPower;
  private double prevEncoder = 0;
  private double currentRPM = 0;  // The most recent rotational speed of the shooter wheels
  private double previousRPM = 0;  // The shooter wheel speed from the previous cycle
  private ArrayList<String> minMaxRPMs = new ArrayList<String>();
  private boolean isTakeBackHalfEnabled = false;
  private boolean trendingUp = true;  // True if the shooter wheel RPM has been trending upward
  private double localMaxRPM = 0;
  private double localMinRPM = Integer.MAX_VALUE;
  private long currentTime;
  private long prevTime = 0;
  private long prevMinMaxTime = 0;
  private boolean turretLoggingEnabled;
  private boolean isAutoRpmEnabled = false;
  private Average distanceAverager = new Average(5);
 
  public ShooterRPM(LimelightVision limelightVision) {
    this.limelightVision = limelightVision;
    spinMotorEncoder.setDistancePerPulse(1 / TICKS_PER_FLYWHEEL_REVOLUTION);
    spinMotor1.setInverted(true);
    spinMotor2.setInverted(true);
    turretLoggingEnabled = NRGPreferences.ENABLE_TURRET_LOGGING.getValue();
  }

  /** Sets the default command for this subsystem that runs when no other command is active. */
  public void initDefaultCommand() { }

  /**
   * Calculates the current RPM of the shooter flywheel and saves it in {@code currentRPM}.
   * 
   * We initialy tried to measure RPM using Encoder.getRate(), but the resulting RPM was
   * varying a lot because the time period between encoder ticks was so small. Instead,
   * we calculate the RPM manually using Counter.getDistance(), which yields a steadier
   * measurement.
   * 
   * WARNING: will compute innacurate results if called more than once per 20ms cycle.
   */
  private void calculateCurrentRPM() {
    double currentEncoder = spinMotorEncoder.getDistance();
    currentTime = System.nanoTime();
    previousRPM = currentRPM;
    currentRPM = (currentEncoder - prevEncoder) / (currentTime - prevTime) * NANOSECS_PER_MINUTE;
    prevEncoder = currentEncoder;
    prevTime = currentTime;
  }

  public double getActualRPM() {
    return currentRPM;
  }

  /** Updates the flywheel motor controllers using Take-Back-Half closed-loop control. */
  public void updateMotorPowerToSeekGoalRpm() {
    error = goalRPM - currentRPM; // calculate the error
    motorPower += GAIN * error; // integrate the error
    motorPower = MathUtil.clamp(motorPower, 0, 1);
    if (error * previousError < 0) { // if it crossed over the goal RPM
      motorPower = 0.5 * (motorPower + tbh); // then Take Back Half
      tbh = motorPower; // update Take Back Half variable
      previousError = error; // and save the previous error
    }
    setFlyWheel(motorPower);
  }

  public void enableAutoRPM(boolean enable) {
    if (enable) {
      isAutoRpmEnabled = true;
      isTakeBackHalfEnabled = true;
    } else {
      disableTakeBackHalf();
    }
  }

  /** Sets the target speed of the shooter flywheel in revolutions per minute. */
  public void setGoalRPM(double newGoalRPM) {
    if (newGoalRPM == 0) {
      motorPower = 0;
      previousError = 0;
      tbh = 0;
    } else {
      isTakeBackHalfEnabled = true;
      // If we're going from stopped to a positive goal RPM, use full power
      // to spin up the flywheel as quickly as possible, and initialize TBH
      // so that our best-guess-power is used after the first crossover.
      if (goalRPM == 0 && newGoalRPM > 0) {
        motorPower = 1;
        previousError = 1;
        tbh = 2 * guessMotorOutputForRPM(newGoalRPM) - 1;
        prevMinMaxTime = System.nanoTime();
      } else {
        // If the goal RPM is being continuously changed from one non-zero
        // value to another, just let TBH continue to integrate as normal.
      }
    }
    goalRPM = newGoalRPM;
  }

  private void autoSetGoalRPM() {
    distanceAverager.add(limelightDistanceToRPM());
    setGoalRPM(distanceAverager.averaged());
  } 
  
  private double limelightDistanceToRPM() {
    double distance = limelightVision.getDistance();
    double rpm = MathUtil.clamp(2.3778 * distance + 2770.3, MIN_RPM, MAX_RPM);
    return rpm;
  }

  /** Estimates the flywheel motor power needed to maintain a given RPM rate. */
  public double guessMotorOutputForRPM(double RPM) {
    // TODO: replace this dumb linear estimate with something more accurate.
    return MathUtil.clamp(RPM / MAX_RPM, 0, 1);
  }

  /** Sets the flywheel spin motor controllers to the given power. */
  public void setFlyWheel(double power) {
    spinMotor1.set(power);
    spinMotor2.set(power);
    lastMotorPower = power;
  }

  /** Sets the flywheel spin motor controllers to the given voltage. */
  public void setFlyWheelVoltage(double power) {
    spinMotor1.setVoltage(power * 12);
    spinMotor2.setVoltage(power * 12);
    lastMotorPower = power;
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    calculateCurrentRPM();
    if (isTakeBackHalfEnabled) {
      if (DriverStation.getInstance().isDisabled()) {
        this.disableTakeBackHalf();
      } else {
        if (isAutoRpmEnabled) {
          autoSetGoalRPM(); // Continuously adjust RPM based on Limelight distance.
        }
        updateMotorPowerToSeekGoalRpm();
      }
    }
    if (turretLoggingEnabled) {
      trackMinMaxRPM(30);
    }
  }

  /** Resets the ShooterRPM subsystem to its initial state. */
  public void reset() {
    isTakeBackHalfEnabled = false;
    isAutoRpmEnabled = false;
    prevEncoder = 0;
    spinMotorEncoder.reset();
    spinMotor1.disable();
    spinMotor2.disable();
    setGoalRPM(0);
    lastMotorPower = 0;
  }

  public void disableTakeBackHalf() {
    reset();
  }

  /**
   * Logs and adds to the list of minMax values if a new local minimum or naximum is detected in the RPM values.
   * @param toleranceRPM how sensitive the method should be to RPM changes
   */
  public void trackMinMaxRPM(double toleranceRPM) {
    if (trendingUp) {  // RPM is trending upward (or stable)
      if (currentRPM >= localMaxRPM) {
        localMaxRPM = currentRPM;
      } else if (currentRPM < localMaxRPM - toleranceRPM) {  // Reversing direction?
        trendingUp = false;
        localMinRPM = currentRPM;
        String s = String.format("Max rpm: %6.1f time: %5.3f", localMaxRPM, getTimeSincePrevMinMax());
        minMaxRPMs.add(s);
        System.out.println(s);
      }
    } else {  // RPM is trending downward (or stable)
      if (currentRPM <= localMinRPM) {
        localMinRPM = currentRPM;
      } else if (currentRPM > localMinRPM + toleranceRPM) {  // Reversing direction?
        trendingUp = true;
        localMaxRPM = currentRPM;
        String s = String.format("Min rpm: %6.1f time: %5.3f", localMinRPM, getTimeSincePrevMinMax());
        minMaxRPMs.add(s);
        System.out.println(s);
      }
    }
  }

  // Returns the array of local mins and local maxs
  public String[] getMinMaxArray() {
    return minMaxRPMs.toArray(new String[0]);
  }

  // Returns true if currentRPM just became equal to goalRPM
  public boolean justReachedGoalRPM(double tolerance) {
    return Math.abs(currentRPM - goalRPM) <= tolerance && Math.abs(previousRPM - goalRPM) > tolerance;
  }

  // Returns time (in seconds) that has elapsed since the previous RPM local minimum/maximum.
  private double getTimeSincePrevMinMax() {
    double deltaTime = (currentTime - prevMinMaxTime) / NANOSECS_PER_SEC;
    prevMinMaxTime = currentTime;
    return deltaTime;
  }

  public void addShuffleBoardTab() {
    if (!NRGPreferences.SHUFFLEBOARD_SHOOTER_RPM_ENABLED.getValue()) {
      return;
    }

    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    ShuffleboardLayout layout = shooterTab.getLayout("Shooter", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);
    layout.add("Encoder", this.spinMotorEncoder);
    layout.addNumber("Power", () -> this.lastMotorPower);
    layout.addNumber("RPM", () -> this.currentRPM);
    shooterTab.addNumber("RPM", () -> this.currentRPM).withWidget(BuiltInWidgets.kGraph).withPosition(2, 0).withSize(6, 4);
  }
}