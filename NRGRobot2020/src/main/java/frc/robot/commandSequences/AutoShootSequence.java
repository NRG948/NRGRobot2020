package frc.robot.commandSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.AutoFeedToShooter;
import frc.robot.commands.AutoTurret;
import frc.robot.commands.DisableShooterRPM;
import frc.robot.commands.MaintainShooterRPM;
import frc.robot.commands.WaitForMinRPM;
import frc.robot.subsystems.Acquirer;
import frc.robot.subsystems.BallCounter;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShootSequence extends SequentialCommandGroup {
  /**
   * Shoots up to 5 balls in an autonoumous mode.
   */
  public AutoShootSequence(double rpm, ShooterRPM shooterRPM, Turret turret, Feeder feeder, Acquirer acquirer, BallCounter ballCounter) {

    super( 
      new MaintainShooterRPM(shooterRPM).atRpm(rpm).setAndExit()
        .alongWith(new AutoTurret(turret)),
      new WaitForMinRPM(rpm, shooterRPM),
      // Release Ball
      new AutoFeedToShooter(acquirer, feeder, ballCounter),
      // Are you at target rpm?
      new WaitForMinRPM(rpm, shooterRPM),
      // Release Ball
      new AutoFeedToShooter(acquirer, feeder, ballCounter),
      // Are you at target rpm?
      new WaitForMinRPM(rpm, shooterRPM),
      // Release Ball
      new AutoFeedToShooter(acquirer, feeder, ballCounter),
      // Are you at target rpm?
      new WaitForMinRPM(rpm, shooterRPM),
      // Release Ball
      new AutoFeedToShooter(acquirer, feeder, ballCounter),
      // Are you at target rpm?
      new WaitForMinRPM(rpm, shooterRPM),
      // Release Ball
      new AutoFeedToShooter(acquirer, feeder, ballCounter),
      new DisableShooterRPM(shooterRPM)
    );
  }
  
  public AutoShootSequence(double rpm, RobotSubsystems subsystems){
    this(rpm, subsystems.shooterRPM, subsystems.turret, subsystems.feeder, subsystems.acquirer, subsystems.ballCounter);
  }
}
