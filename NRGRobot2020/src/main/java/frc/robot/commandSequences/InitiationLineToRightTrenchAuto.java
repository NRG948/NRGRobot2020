package frc.robot.commandSequences;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Delay;
import frc.robot.commands.FollowWaypoints;
import frc.robot.commands.SetStartPosition;
import frc.robot.subsystems.Drive;

/**
 * Autonomous command sequence moving the robot from the initiation line to
 * the right trench, and then into shooting position.
 */
public class InitiationLineToRightTrenchAuto extends SequentialCommandGroup {
  /**
   * Creates a new InitiationLineToRightTrenchAuto.
   */
  public InitiationLineToRightTrenchAuto(Drive drive) {
    super(new SetStartPosition(drive, new Pose2d(3.473, -7.2, new Rotation2d(0))),
          new FollowWaypoints(drive,
                              // Starting pose
                              new Pose2d(3.43, -7.2, new Rotation2d(0)),
                              // Waypoints
                              List.of(new Translation2d(5.068, -6.809)),
                              // Ending pose
                              new Pose2d(6.2, -7.2, new Rotation2d(Math.toRadians(-45))),
                              // Drive forward
                              false),
          new Delay(0.5), 
          new FollowWaypoints(drive,
                              // Starting pose
                              new Pose2d(6.22, -7.2, new Rotation2d(Math.toRadians(-45))),
                              // Waypoints
                              List.of(new Translation2d(4.648, -4.236)),
                              // Ending pose
                              new Pose2d(4.549, -2.567, new Rotation2d(Math.toRadians(-90))),
                              // Drive backward
                              true));
  }
}
