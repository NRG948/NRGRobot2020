package frc.robot.commandSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Delay;
import frc.robot.commands.DisableShooterRPM;
import frc.robot.commands.SetHoodPosition;
import frc.robot.commands.SetLimelightHorizontalSkew;
import frc.robot.commands.StopTurretAnglePID;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class StopAutoShootSequence extends SequentialCommandGroup {
  /**
   * Creates a new StopAutoShootSequence.
   */
  public StopAutoShootSequence(RobotSubsystems subsystems) {
    super(new Delay(0.25)
    .andThen(new DisableShooterRPM(subsystems.shooterRPM)
      .alongWith(new StopTurretAnglePID(subsystems.turret), 
                 new SetHoodPosition(subsystems.hood, 2), 
                 new SetLimelightHorizontalSkew(subsystems.turret, 0))));
  }
}
