package frc.robot.commandSequences;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.AcquireNumberOfBalls;
import frc.robot.commands.AutoDriveOnHeading;
import frc.robot.commands.AutoFeeder;
import frc.robot.commands.Delay;
import frc.robot.commands.SetAcquirerState;
import frc.robot.subsystems.AcquirerPiston.State;

/**
 * Autonomous sequence for when our robot just wants to stay out of everyone's way.
 * 
 * We roll forward off the line to get the 5 points.
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
public class InitiationLineRollForward extends SequentialCommandGroup {  
  public static final Pose2d INITIAL_POSITION = new Pose2d(); // TODO: determine correct starting position

/**
   * Creates a new InitiationLineRollForward.
   */
  public InitiationLineRollForward(RobotSubsystems subsystems, float delay) {
    // Start anywhere on the initiation line, move forward one meter.
    super(new Delay(delay),
          new SetAcquirerState(subsystems.acquirerPiston, State.RETRACT), 
          new AutoDriveOnHeading(subsystems.drive).withMaxPower(0.5).forMeters(1));
  }
}
