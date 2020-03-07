package frc.robot.commandSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.AutoFeedToShooter;
import frc.robot.commands.AutoRPM;
import frc.robot.commands.AutoTurret;
import frc.robot.commands.Delay;
import frc.robot.commands.DisableShooterRPM;
import frc.robot.commands.MaintainShooterRPM;
import frc.robot.commands.SetHoodPosition;
import frc.robot.commands.SetLimelightHorizontalSkew;
import frc.robot.commands.WaitForMinRPM;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShootSequence extends SequentialCommandGroup {
  /**
   * Shoots up to 5 balls in an autonoumous mode.
   */
  public AutoShootSequence(RobotSubsystems subsystems, double rpm, double hoodPosition, double skew){
    super(
      new MaintainShooterRPM(subsystems.shooterRPM).atRpm(rpm).setAndExit()
      // new AutoRPM(subsystems.shooterRPM, true)
        .alongWith(new AutoTurret(subsystems.turret).usingLimelight(), 
                   new SetHoodPosition(subsystems.hood, hoodPosition),
                   new SetLimelightHorizontalSkew(subsystems.turret, skew)),
      new WaitForMinRPM(rpm, subsystems.shooterRPM),
      // Release Ball
      new AutoFeedToShooter(subsystems.acquirer, subsystems.feeder, subsystems.ballCounter),
      // Are you at target rpm?
      new WaitForMinRPM(rpm, subsystems.shooterRPM),
      // Release Ball
      new AutoFeedToShooter(subsystems.acquirer, subsystems.feeder, subsystems.ballCounter),
      // Are you at target rpm?
      new WaitForMinRPM(rpm, subsystems.shooterRPM),
      // Release Ball
      new AutoFeedToShooter(subsystems.acquirer, subsystems.feeder, subsystems.ballCounter),
      // Are you at target rpm?
      new WaitForMinRPM(rpm, subsystems.shooterRPM),
      // Release Ball
      new AutoFeedToShooter(subsystems.acquirer, subsystems.feeder, subsystems.ballCounter),
      // Are you at target rpm?
      new WaitForMinRPM(rpm, subsystems.shooterRPM),
      // Release Ball
      new AutoFeedToShooter(subsystems.acquirer, subsystems.feeder, subsystems.ballCounter)
    );
  }
}
