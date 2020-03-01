
package frc.robot.commandSequences;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.AcquireNumberOfBalls;
import frc.robot.commands.AutoFeeder;
import frc.robot.commands.DriveToFuelCell;
import frc.robot.commands.FollowWaypoints;
import frc.robot.commands.MaintainShooterRPM;
import frc.robot.commands.SetAcquirerState;
import frc.robot.commands.SetHoodPosition;
import frc.robot.commands.SetLimelightHorizontalSkew;
import frc.robot.commands.SetRaspberryPiPipeline;
import frc.robot.commands.StopTurretAnglePID;
import frc.robot.commands.TurnTurretToAngle;
import frc.robot.subsystems.AcquirerPiston.State;
import frc.robot.subsystems.RaspberryPiVision.PipelineRunner;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class InitiationLineToRightTrenchAuto extends SequentialCommandGroup {
  public static final Pose2d INITIAL_POSITION = new Pose2d(3.676, -0.686, new Rotation2d(0));
  private static final Translation2d FIRST_PATH_WAYPOINT = new Translation2d(4.5, -0.686);
  private static final Pose2d FIRST_PATH_END_POSITION = new Pose2d(5.784, -0.686, new Rotation2d(0));
  private static final Pose2d SECOND_PATH_START_POSITION = new Pose2d(9.245, -0.686, new Rotation2d(0));
  private static final Translation2d SECOND_PATH_WAYPOINT = new Translation2d(8.245, -0.686);
  private static final Pose2d SECOND_PATH_END_POSITION = new Pose2d( 6.745, -1.3, new Rotation2d(0));
  /**
   * Creates a new InitiationLineToLeftTrenchAuto.
   */
  public InitiationLineToRightTrenchAuto(RobotSubsystems subsystems) {
    super(
      new FollowWaypoints(subsystems.drive,
                          INITIAL_POSITION,
                          List.of(FIRST_PATH_WAYPOINT), 
                          FIRST_PATH_END_POSITION, 
                          false)
        .alongWith(new SetAcquirerState(subsystems.acquirerPiston, State.EXTEND),
                   new SetRaspberryPiPipeline(subsystems.raspPi, PipelineRunner.FUEL_CELL),
                   new TurnTurretToAngle(subsystems.turret, 77),
                   new SetHoodPosition(subsystems.hood, 72),
                   new AcquireNumberOfBalls(subsystems.acquirer, subsystems.ballCounter).withRelativeCount(1).withTimeout(3),
                   new AutoFeeder(subsystems.ballCounter, subsystems.feeder),
                   new MaintainShooterRPM(subsystems.shooterRPM).atRpm(2500).setAndExit()),
      new SetLimelightHorizontalSkew(subsystems.turret, -3),
      new AutoShootSequence(4000, subsystems),
      new AutoDriveToFuelCell(subsystems,3).alongWith(new SetHoodPosition(subsystems.hood ,2)),
      new FollowWaypoints(subsystems.drive,
                          SECOND_PATH_START_POSITION,
                          List.of(SECOND_PATH_WAYPOINT),
                          SECOND_PATH_END_POSITION,
                          true),
      new SetHoodPosition(subsystems.hood,72),
      new AutoShootSequence(4200, subsystems),
      new StopTurretAnglePID(subsystems.turret)
      );
  }
}
