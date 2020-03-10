package frc.robot.commandSequences;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.AutoDriveOnHeading;
import frc.robot.commands.Delay;
import frc.robot.utilities.Logger;

/**
 * Autonomous sequence for when our robot just wants to stay out of everyone's way.
 * 
 * We roll forward off the line to get the 5 points.
 * 
 * Use arrows to mark robot positions and (acquirer) orientations: <>^v\/
 *   
 *        /-------------------------------------------------\  
 *       /       ¦      ¦              ¦¦¦◦   ¦      ¦       \ 
 *      /        ¦      ¦   ◦   ◦   ◦  ¦¦¦    ¦      ¦        \ 
 *     /         ¦      ¦              ¦¦¦◦   ¦      ¦         \
 *    /          ¦      +---------------------+      ¦          \
 * T ¦\          ¦           ◦                       ¦          /¦ L
 * g ¦ }         ¦         ◦                         ¦         { ¦ d
 * t ¦/          ¦                         ◦         ¦          \¦ r
 *   ¦           ¦        ◦                 ◦        ¦           ¦
 *   ¦           ¦         ◦                 ◦       ¦           ¦
 * L ¦\          ¦          ◦                        ¦          /¦ T
 * d ¦ }         ¦                         ◦         ¦         { ¦ g
 * r ¦/          ¦                       ◦           ¦          \¦ t
 *    \          ¦      +---------------------+      ¦          /
 *     \         ¦      ¦   ◦¦¦¦              ¦      ¦         /
 *      \        ¦      ¦    ¦¦¦  ◦   ◦   ◦   ¦      ¦        /
 *       \       ¦      ¦   ◦¦¦¦              ¦      ¦       /
 *        \-------------------------------------------------/
 */
public class InitiationLineRollForward extends SequentialCommandGroup {  
  // This is needed by RobotContainer because it sets the initial position of the robot.
  public static final Pose2d INITIAL_POSITION = new Pose2d();

  /**
   * InitiationLineRollForward moves one meter forward off the initiation line.
   */
  public InitiationLineRollForward(RobotSubsystems subsystems, float delay, double distance) {
    // Start anywhere on the initiation line, move forward one meter.
    super(new Delay(delay),
          new AutoDriveOnHeading(subsystems.drive).withMaxPower(0.5).forMeters(distance));
  }

  @Override
  public void initialize(){
    Logger.commandInit(this);
    super.initialize();
  }
  
  @Override
  public void end(boolean interrupted){
    super.end(interrupted);
    Logger.commandEnd(this, interrupted);
  }
}
