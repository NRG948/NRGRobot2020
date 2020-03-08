package frc.robot.commandSequences;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.AcquireNumberOfBalls;
import frc.robot.commands.AutoFeeder;
import frc.robot.commands.Delay;
import frc.robot.commands.FollowWaypoints;
import frc.robot.commands.SetAcquirerState;
import frc.robot.commands.TurnTurretToAngle;
import frc.robot.subsystems.AcquirerPistons.State;

/**
 * Autonomous command sequence moving the robot from the initiation line to the
 * left (aka opponent) trench, and then into shooting position.
 *
 * Use arrows to mark robot positions and (acquirer) orientations: <>^v\/
 *   
 *        /-------------------------------------------------\  
 *       /       ¦      ¦              ¦¦¦◦   ¦      ¦       \ 
 *      /        ¦      ¦   ◦   ◦   ◦  ¦¦¦^D  ¦     <B        \ 
 *     /         ¦      ¦              ¦¦¦◦   ¦      ¦         \
 *    /          ¦      +---------------------+\C    ¦          \
 * T ¦\          ¦           ◦                       ¦          /¦ L
 * g ¦ }         ¦         ◦                         ¦         { ¦ d
 * t ¦/          ¦                         ◦         ¦          \¦ r
 *   ¦           ¦        ◦                 ◦        ¦           ¦
 *   ¦           ¦         ◦                 ◦       ¦           ¦
 * L ¦\          ¦          ◦                        ¦          /¦ T
 * d ¦ }         ¦                         ◦       ^F¦         { ¦ g
 * r ¦/          ¦                       ◦           ¦          \¦ t
 *    \          ¦      +---------------------+      ¦          /
 *     \         ¦      ¦   ◦¦¦¦              ¦      ¦         /
 *      \        ¦      ¦    ¦¦¦  ◦   ◦   ◦   ¦      ¦        /
 *       \       ¦      ¦   ◦¦¦¦              ¦      ¦       /
 *        \-------------------------------------------------/
 */
public class InitiationLineToLeftTrenchAuto extends SequentialCommandGroup {
  /**
   *
   */
  public static final Pose2d INITIAL_POSITION = new Pose2d(3.676, -7.2, new Rotation2d(0));
  private static final Pose2d FIRST_PATH_END_POSITION = new Pose2d(6.4, -7.45, new Rotation2d(Math.toRadians(-65)));
  private static final List<Translation2d> FIRST_PATH_WAYPOINTS = List.of(new Translation2d(5.068, -6.6));
  private static final List<Translation2d> SECOND_PATH_WAYPOINTS = List.of(new Translation2d(4.648, -4.236));
  private static final Pose2d SECOND_PATH_END_POSITION = new Pose2d(4.549, -2.867, new Rotation2d(Math.toRadians(-90)));
  
  /**
   * Creates a new InitiationLineToRightTrenchAuto.
   */
  public InitiationLineToLeftTrenchAuto(RobotSubsystems subsystems, float delay) {
    // Start on the initiation line, centered between two opponent's balls (A)
    super(new Delay(delay),
          new SetAcquirerState(subsystems.acquirerPiston, State.EXTEND), 
          new FollowWaypoints(subsystems.drive,
                              INITIAL_POSITION, // Starting pose  (B)
                              FIRST_PATH_WAYPOINTS,  // Waypoints (C)
                              FIRST_PATH_END_POSITION, // Ending pose (D)
                              false)  // Drive forward
            // while driving, also make sure we're ready to load more balls
            // turn turret to maximize carrying capacity
            .alongWith(new TurnTurretToAngle(subsystems.turret, 77),
                       // we hope to pick up 2 ball, but give up after 3 seconds
                       new AcquireNumberOfBalls(subsystems.acquirer, subsystems.ballCounter).withRelativeCount(2).withTimeout(3), 
                       // make sure our starting state has the initial ball breaking the beam
                       new AutoFeeder(subsystems.ballCounter, subsystems.feeder)),
          // retract aquirer before driving to next point
          new SetAcquirerState(subsystems.acquirerPiston, State.RETRACT),
          // drive to ideal shooting position (F)
          new FollowWaypoints(subsystems.drive,
                              FIRST_PATH_END_POSITION,  // Starting pose (D)
                              SECOND_PATH_WAYPOINTS,  // Waypoints (E)
                              SECOND_PATH_END_POSITION,  // Ending pose (F)
                              true)  // Drive backward
            // turn turret in the rough direction of the target
            .alongWith(new TurnTurretToAngle(subsystems.turret, 100)),
            // FIRE! (with auto-targetting)
          new AutoShootSequence(subsystems, 4000, 72, 0));
  }
}
