package frc.robot.commandSequences;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.AcquireNumberOfBalls;
import frc.robot.commands.AutoFeeder;
import frc.robot.commands.AutoTurnToHeading;
import frc.robot.commands.FollowWaypoints;
import frc.robot.commands.SetAcquirerState;
import frc.robot.commands.SetStartPosition;
import frc.robot.commands.TurnTurretToAngle;
import frc.robot.subsystems.AcquirerPiston.State;

/**
 * Autonomous command sequence moving the robot from the initiation line to
 * the left trench.
 */
public class InitiationLineToShieldGeneratorAuto extends SequentialCommandGroup {
  /**
   * Creates a new InitiationLineToLeftTrenchAuto.
   */
  public InitiationLineToShieldGeneratorAuto(RobotSubsystems subsystems) {
    super(
      new SetStartPosition(subsystems.drive, new Pose2d(3.3, -0.786, new Rotation2d(0))), 
      new SetAcquirerState(subsystems.acquirerPiston, State.EXTEND),
      new FollowWaypoints(subsystems.drive,
                          // Starting pose
                          new Pose2d( 3.3, -0.786, new Rotation2d(0)),
                          // Waypoint
                          List.of(new Translation2d(4.821, -1.553)),
                          // Ending pose
                          new Pose2d(6.107, -2.987, new Rotation2d(Math.toRadians(-120))),
                          // Drive forward
                          false)
          .alongWith(new AcquireNumberOfBalls(subsystems.acquirer, subsystems.ballCounter).withRelativeCount(2).withTimeout(5), 
                     new AutoFeeder(subsystems.ballCounter, subsystems.feeder)),
        new SetAcquirerState(subsystems.acquirerPiston, State.RETRACT),
        new AutoTurnToHeading(subsystems.drive).toHeading(-75).withTolerance(2).withMaxPower(0.8)
          .alongWith(new TurnTurretToAngle(subsystems.turret, 130)),
        new AutoShootSequence(4000, subsystems));
  }
}
