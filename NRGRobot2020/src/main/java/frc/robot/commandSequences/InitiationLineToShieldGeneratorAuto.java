package frc.robot.commandSequences;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AcquireNumberOfBalls;
import frc.robot.commands.AutoFeeder;
import frc.robot.commands.AutoTurnToHeading;
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

/**
 * Autonomous command sequence moving the robot from the initiation line to
 * the left trench.
 */
public class InitiationLineToShieldGeneratorAuto extends SequentialCommandGroup {
  /**
   * Creates a new InitiationLineToLeftTrenchAuto.
   */
  public InitiationLineToShieldGeneratorAuto(Drive drive, Acquirer acquirer, Feeder feeder, BallCounter ballCounter,
  ShooterRPM shooterRPM, Turret turret, LimelightVision limelightVision, AcquirerPiston acquirerPiston) {
    super(
      new SetStartPosition(drive, new Pose2d(3.3, -0.786, new Rotation2d(0))), 
      new SetAcquirerState(acquirerPiston, State.EXTEND),
      new FollowWaypoints(drive,
                          // Starting pose
                          new Pose2d( 3.3, -0.786, new Rotation2d(0)),
                          // Waypoint
                          List.of(new Translation2d(4.821, -1.553)),
                          // Ending pose
                          new Pose2d(6.107, -2.987, new Rotation2d(Math.toRadians(-120))),
                          // Drive forward
                          false)
          .alongWith(new AcquireNumberOfBalls(acquirer, ballCounter).withRelativeCount(2).withTimeout(5))
          .alongWith(new AutoFeeder(ballCounter, feeder))
          .andThen(new SetAcquirerState(acquirerPiston, State.RETRACT)),
        new AutoTurnToHeading(drive).toHeading(-75).withTolerance(2).withMaxPower(0.8)
          .alongWith(new TurnTurretToAngle(turret, 130)),
        new AutoShootSequence(4000, shooterRPM, turret, feeder, acquirer, ballCounter, limelightVision));
  }
}
