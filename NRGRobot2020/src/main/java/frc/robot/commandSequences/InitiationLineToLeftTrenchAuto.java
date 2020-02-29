package frc.robot.commandSequences;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.AcquireNumberOfBalls;
import frc.robot.commands.AutoFeeder;
import frc.robot.commands.FollowWaypoints;
import frc.robot.commands.SetAcquirerState;
import frc.robot.commands.SetStartPosition;
import frc.robot.commands.TurnTurretToAngle;
import frc.robot.subsystems.AcquirerPiston.State;

/**
 * Autonomous command sequence moving the robot from the initiation line to the
 * right trench, and then into shooting position.
 */
public class InitiationLineToLeftTrenchAuto extends SequentialCommandGroup {
  /**
   * Creates a new InitiationLineToRightTrenchAuto.
   */
  public InitiationLineToLeftTrenchAuto(RobotSubsystems subsystems) {
    super(new SetStartPosition(subsystems.drive, new Pose2d(3.676, -7.2, new Rotation2d(0))), 
          new SetAcquirerState(subsystems.acquirerPiston, State.EXTEND), 
          new FollowWaypoints(subsystems.drive,
                              // Starting pose
                              new Pose2d(3.43, -7.2, new Rotation2d(0)),
                              // Waypoints
                              List.of(new Translation2d(5.068, -6.6)),
                              // Ending pose
                              new Pose2d(6.4, -7.45, new Rotation2d(Math.toRadians(-65))),
                              // Drive forward
                              false)
            .alongWith(new TurnTurretToAngle(subsystems.turret, 77),
                       new AcquireNumberOfBalls(subsystems.acquirer, subsystems.ballCounter).withRelativeCount(2).withTimeout(3), 
                       new AutoFeeder(subsystems.ballCounter, subsystems.feeder)),
          new SetAcquirerState(subsystems.acquirerPiston, State.RETRACT),
          new FollowWaypoints(subsystems.drive,
                              // Starting pose
                              new Pose2d(6.4, -7.45, new Rotation2d(Math.toRadians(-65))),
                              // Waypoints
                              List.of(new Translation2d(4.648, -4.236)),
                              // Ending pose
                              new Pose2d(4.549, -2.867, new Rotation2d(Math.toRadians(-90))),
                              // Drive backward
                              true)
            .alongWith(new TurnTurretToAngle(subsystems.turret, 100)),
          new AutoShootSequence(4000, subsystems));
  }
}
