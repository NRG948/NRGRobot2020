package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Subsystem which controls the vectoring ball intake rollers that extends beyond the bumper.
 * 
 * The pistons that extend/retract the acquirer are a separate subsystem.
 */
public class Acquirer extends SubsystemBase {

  private static final double MAX_ACQUIRER_POWER = 0.5;

  private Victor acquirerMotor = new Victor(2);

  /** Creates a new Acquirer. */
  public Acquirer() {
  }

  public void rawAcquirer(double power){
    power = MathUtil.clamp(power, -MAX_ACQUIRER_POWER, MAX_ACQUIRER_POWER);
    acquirerMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}