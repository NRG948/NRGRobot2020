
package frc.robot.commandSequences;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AcquireNumberOfBalls;
import frc.robot.commands.AutoFeeder;
import frc.robot.commands.FollowWaypoints;
import frc.robot.commands.SetAcquirerState;
import frc.robot.commands.SetStartPosition;
import frc.robot.commands.TurnTurretToAngle;
import frc.robot.subsystems.Acquirer;
import frc.robot.subsystems.AcquirerPiston;
import frc.robot.subsystems.BallCounter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.AcquirerPiston.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class InitiationLineToRightTrenchAuto extends SequentialCommandGroup {
  private static final Pose2d INITIAL_POSITION = new Pose2d(3.676, 0.686, new Rotation2d(0));
  private static final Translation2d FIRST_PATH_WAYPOINT = new Translation2d(4.5, 0.686);
  private static final Pose2d FIRST_PATH_END_POSITION = new Pose2d(5.784, 0.686, new Rotation2d(0));
  /**
   * Creates a new InitiationLineToLeftTrenchAuto.
   */
  public InitiationLineToRightTrenchAuto(Drive drive, Acquirer acquirer, Feeder feeder, BallCounter ballCounter,
  ShooterRPM shooterRPM, Turret turret, LimelightVision limelightVision, AcquirerPiston acquirerPiston) {
    super(new FollowWaypoints(drive,
                              INITIAL_POSITION,
                              List.of(FIRST_PATH_WAYPOINT), 
                              FIRST_PATH_END_POSITION, 
                              false)
        .alongWith(new SetAcquirerState(acquirerPiston, State.EXTEND),
                   new AcquireNumberOfBalls(acquirer, ballCounter).withRelativeCount(1).withTimeout(3),
                   new AutoFeeder(ballCounter, feeder)));
  }
}
