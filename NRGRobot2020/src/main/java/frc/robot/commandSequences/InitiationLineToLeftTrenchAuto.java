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

/*
  Use arrows to mark robot positions and (aquirer) orientations: ↑↓←→↖↗↙↘
  
       ╱═════════════╤═════════════════════╤═════════════╲  
      ╱       ⁞      │              ░░░₀   │      ⁞       ╲ 
     ╱        ⁞      │   ○   ○   ○  ░░░↖D  │      ←A       ╲ 
    ╱         ⁞      │              ░░░⁰   │      ⁞         ╲
   ╱          ⁞      └─────────────────────┘↖     ⁞          ╲
T ║╲          ⁞                                   ⁞          ╱║ L
g ║ }         ⁞         ₀ ⁰                       ⁞         { ║ d
t ║╱          ⁞       ₀                 ₀         ⁞          ╲║ r
  ║           ⁞        ₀                 ₀        ⁞           ║
  ║           ⁞         ₀                 ₀     ↑F⁞           ║
L ║╲          ⁞                                   ⁞          ╱║ T
d ║ }         ⁞                       ₀ ⁰         ⁞         { ║ g
r ║╱          ⁞                                   ⁞          ╲║ t
   ╲          ⁞      ┌─────────────────────┐      ⁞          ╱
    ╲         ⁞      │   ₀░░░              │      ⁞         ╱
     ╲        ⁞      │    ░░░  ○   ○   ○   │      ⁞        ╱
      ╲       ⁞      │   ⁰░░░              │      ⁞       ╱
       ╲═════════════╧═════════════════════╧═════════════╱
*/

/**
 * Autonomous command sequence moving the robot from the initiation line to the
 * left (aka opponent) trench, and then into shooting position.
 */
public class InitiationLineToLeftTrenchAuto extends SequentialCommandGroup {
  /**
   * Creates a new InitiationLineToRightTrenchAuto.
   */
  public InitiationLineToLeftTrenchAuto(RobotSubsystems subsystems) {
    // Start on the initiation line, centered between two opponent's balls (A)
    super(new SetStartPosition(subsystems.drive, new Pose2d(3.676, -7.2, new Rotation2d(0))), 
          new SetAcquirerState(subsystems.acquirerPiston, State.EXTEND), 
          new FollowWaypoints(subsystems.drive,
                              new Pose2d(3.43, -7.2, new Rotation2d(0)), // Starting pose  (B)
                              List.of(new Translation2d(5.068, -6.6)),  // Waypoints (C)
                              new Pose2d(6.4, -7.45, new Rotation2d(Math.toRadians(-65))), // Ending pose (D)
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
                              new Pose2d(6.4, -7.45, new Rotation2d(Math.toRadians(-65))),  // Starting pose (D)
                              List.of(new Translation2d(4.648, -4.236)),  // Waypoints (E)
                              new Pose2d(4.549, -2.867, new Rotation2d(Math.toRadians(-90))),  // Ending pose (F)
                              true)  // Drive backward
            // turn turret in the rough direction of the target
            .alongWith(new TurnTurretToAngle(subsystems.turret, 100)),
            // FIRE! (with auto-targetting)
          new AutoShootSequence(4000, subsystems));
  }
}
