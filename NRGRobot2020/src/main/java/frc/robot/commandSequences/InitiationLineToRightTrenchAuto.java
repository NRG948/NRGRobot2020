
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
import frc.robot.commands.MaintainShooterRPM;
import frc.robot.commands.SetAcquirerState;
import frc.robot.commands.SetHoodPosition;
import frc.robot.commands.SetLimelightHorizontalSkew;
import frc.robot.commands.SetRaspberryPiPipeline;
import frc.robot.commands.StopTurretAnglePID;
import frc.robot.commands.TurnTurretToAngle;
import frc.robot.subsystems.AcquirerPistons.State;
import frc.robot.subsystems.RaspberryPiVision.PipelineRunner;
import frc.robot.utilities.Logger;

/**
 * Autonomous command sequence moving the robot from the initiation line to the
 * right (aka alliance) trench, pick up a single ball and shoot four. Then, move
 * through the trech picking up three balls, return to shooting position and
 * shoot four more.
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
 *      \        ¦      ¦  <C¦¦¦  ◦   ◦ D>◦<B ¦     <A        /
 *       \       ¦      ¦   ◦¦¦¦              ¦      ¦       /
 *        \-------------------------------------------------/
 */
public class InitiationLineToRightTrenchAuto extends SequentialCommandGroup {
  public static final Pose2d INITIAL_POSITION = new Pose2d(3.676, -0.686, new Rotation2d(0));
  private static final List<Translation2d> FIRST_PATH_WAYPOINT = List.of(new Translation2d(4.5, -0.686));
  private static final Pose2d FIRST_PATH_END_POSITION = new Pose2d(5.784, -0.686, new Rotation2d(0));
  private static final Pose2d SECOND_PATH_START_POSITION = new Pose2d(9.245, -0.686, new Rotation2d(0));
  private static final List<Translation2d> SECOND_PATH_WAYPOINT = List.of(new Translation2d(8.245, -0.686));
  private static final Pose2d SECOND_PATH_END_POSITION = new Pose2d( 6.745, -1.3, new Rotation2d(0));
  /**
   * Creates a new InitiationLineToLeftTrenchAuto.
   */
  public InitiationLineToRightTrenchAuto(RobotSubsystems subsystems, float delay) {
    super(
      new MaintainShooterRPM(subsystems.shooterRPM).atRpm(2500).setAndExit(),
      new Delay(delay + 0.2),
      /* Start on the initiation line, centered in line with the balls on our
       * alliance's trench (A) and move toward the first ball (B). At the same time,
       * attempt to acquire 1 ball and warm up the shooter.
       */
      new FollowWaypoints(subsystems.drive,
                          INITIAL_POSITION,
                          FIRST_PATH_WAYPOINT, 
                          FIRST_PATH_END_POSITION, 
                          false)
        .alongWith(new SetAcquirerState(subsystems.acquirerPiston, State.EXTEND),
                   new SetRaspberryPiPipeline(subsystems.raspPi, PipelineRunner.FUEL_CELL),
                   new TurnTurretToAngle(subsystems.turret, 77),
                   new SetHoodPosition(subsystems.hood, 72),
                   new AcquireNumberOfBalls(subsystems.acquirer, subsystems.ballCounter).withRelativeCount(1).withTimeout(3),
                   new AutoFeeder(subsystems.ballCounter, subsystems.feeder),
                   new MaintainShooterRPM(subsystems.shooterRPM).atRpm(2500).setAndExit()),
      // TODO Use the limelight crosshair adjustement feature to adjust the skew
      new SetLimelightHorizontalSkew(subsystems.turret, -3),
      // Shoot all four balls.
      new AutoShootSequence(subsystems, 3550, 72, -1.5),
      // Stop the AutoShootSequence
      new StopAutoShootSequence(subsystems),
      // Continue to drive toward the fuel cells attempting to pick up three of them (C).
      // At the same time, lower the hood so that we can pass under the control panel.
      new AutoDriveToFuelCell(subsystems, 3).alongWith(new SetHoodPosition(subsystems.hood, 2)),
      // Drive back to shooting position. (D)
      new FollowWaypoints(subsystems.drive,
                          SECOND_PATH_START_POSITION,
                          SECOND_PATH_WAYPOINT,
                          SECOND_PATH_END_POSITION,
                          true),
      // Return the hood to shooting position.
      new SetHoodPosition(subsystems.hood,72),
      // Shoot all three balls.
      new AutoShootSequence(subsystems, 4200, 72, -1),
      // Stop the AutoShootSequence
      new StopAutoShootSequence(subsystems),
      // TODO Stop continous turret PID in AutoShootSequence. 
      new StopTurretAnglePID(subsystems.turret)
      );
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
