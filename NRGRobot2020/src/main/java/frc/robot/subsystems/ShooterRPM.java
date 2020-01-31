package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.MathUtil;

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
  private static final double MAX_RPM = 4200; // figure out actual max rpm
  private static final double TICKS_PER_FLYWHEEL_REVOLUTION = 645;
  private static final double NANOSECS_PER_MINUTE = 60 * 1000000000.0;
  
  private final Victor spinMotor1 = new Victor(0);
  private final Victor spinMotor2 = new Victor(1);
  private final Counter spinMotorEncoder = new Counter(4);

  private double goalRPM = 0;
  private double error = 0;
  private double previousError = 0;
  private double tbh = 0; // Take-back-half value
  private double motorPower = 0;
  private double lastMotorPower;
  private double prevEncoder = 0;
  private double currentRPM = 0;
  private long prevTime = 0;

  public ShooterRPM() {
    spinMotorEncoder.setDistancePerPulse(1 / TICKS_PER_FLYWHEEL_REVOLUTION);
    spinMotor1.setInverted(true);
    spinMotor2.setInverted(true);
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
   * WARNING: will cause innacurate results if called more than once per 20ms cycle.
   */
  public void calculateCurrentRPM() {
    double currentEncoder = spinMotorEncoder.getDistance();
    long currentTime = System.nanoTime();
    currentRPM = (currentEncoder - prevEncoder) / (currentTime - prevTime) * NANOSECS_PER_MINUTE;
    prevEncoder = currentEncoder;
    prevTime = currentTime;
  }

  /** Updates the flywheel motor controllers using Take-Back-Half closed-loop control. */
  public void updateRPM() {
    error = goalRPM - currentRPM; // calculate the error
    motorPower += GAIN * error; // integrate the error
    motorPower = MathUtil.clamp(motorPower, 0, 1);
    if (error * previousError < 0) { // if it crossed over the goal RPM
      motorPower = 0.5 * (motorPower + tbh); // then Take Back Half
      tbh = motorPower; // update Take Back Half variable
      previousError = error; // and save the previous error
    }
    setFlyWheel(motorPower);
    updateDashBoard();
  }

  /** Sets the target speed of the shooter flywheel in revolutions per minute. */
  public void setGoalRPM(double newGoalRPM) {
    if (newGoalRPM == 0) {
      motorPower = 0;
      previousError = 0;
      tbh = 0;
    } else {
      // If we're going from stopped to a positive goal RPM, use full power
      // to spin up the flywheel as quickly as possible, and initialize TBH
      // so that our best-guess-power is used after the first crossover.
      if (goalRPM == 0 && newGoalRPM > 0) {
        motorPower = 1;
        previousError = 1;
        tbh = 2 * guessMotorOutputForRPM(newGoalRPM) - 1;
      } else {
        // If the goal RPM is being continuously changed from one non-zero
        // value to another, just let TBH continue to integrate as normal.
      }
    }
    goalRPM = newGoalRPM;
  }

  /** Estimates the flywheel motor power needed to maintain a given RPM rate. */
  private double guessMotorOutputForRPM(double RPM) {
    // TODO: replace this dumb linear estimate with something more accurate.
    return MathUtil.clamp(RPM / MAX_RPM, 0, 1);
  }

  /** Sets the flywheel spin motor controllers to the given power. */
  public void setFlyWheel(double power) {
    spinMotor1.set(power);
    spinMotor2.set(power);
    lastMotorPower = power;
  }

  /** Sends important subsystem data to the SmartDashboard for monitoring and deubgging. */
  public void updateDashBoard() {
    SmartDashboard.putNumber("ShooterRPM/Raw", spinMotorEncoder.get());
    SmartDashboard.putNumber("ShooterRPM/Distance", spinMotorEncoder.getDistance());
    SmartDashboard.putNumber("ShooterRPM/RPM", currentRPM);
    SmartDashboard.putNumber("ShooterRPM/error", error);
    SmartDashboard.putNumber("ShooterRPM/power", lastMotorPower);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    calculateCurrentRPM();
  }

  /** Resets the ShooterRPM subsystem to its initial state. */
  public void reset() {
    spinMotorEncoder.reset();
    spinMotor1.disable();
    spinMotor2.disable();
    setGoalRPM(0);
    lastMotorPower = 0;
    prevEncoder = 0;
    prevTime = System.nanoTime();
  }
}