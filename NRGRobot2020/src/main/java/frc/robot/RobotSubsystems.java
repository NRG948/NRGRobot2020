package frc.robot;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Acquirer;
import frc.robot.subsystems.AcquirerPiston;
import frc.robot.subsystems.AddressableLEDs;
import frc.robot.subsystems.BallCounter;
import frc.robot.subsystems.ClimberPiston;
import frc.robot.subsystems.ClimberWinch;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Gearbox;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.RaspberryPiVision;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.subsystems.Turret;

/**
 * Add your docs here.
 */
public class RobotSubsystems {
  public final AddressableLEDs leds = new AddressableLEDs();
  public final Drive drive = new Drive();
  public final Gearbox gearbox = new Gearbox();
  public final Acquirer acquirer = new Acquirer();
  public final AcquirerPiston acquirerPiston = new AcquirerPiston();
  public final Feeder feeder = new Feeder();
  public final LimelightVision limelightVision = new LimelightVision();
  public final Turret turret = new Turret(limelightVision);
  public final Hood hood = new Hood();
  public final ShooterRPM shooterRPM = new ShooterRPM(limelightVision);
  public final RaspberryPiVision raspPi = new RaspberryPiVision();
  public final BallCounter ballCounter = new BallCounter();
  public final ClimberWinch climberWinch = new ClimberWinch();
  public final ClimberPiston climberPiston = new ClimberPiston();

  public Subsystem[] getAll(){
    return new Subsystem[]{leds, drive, gearbox, acquirer, acquirerPiston, feeder, turret, hood, shooterRPM, raspPi, ballCounter, climberWinch, climberPiston};
  }
}
