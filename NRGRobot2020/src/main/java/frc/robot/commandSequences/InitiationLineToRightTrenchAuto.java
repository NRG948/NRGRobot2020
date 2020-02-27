package frc.robot.commandSequences;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AcquireNumberOfBalls;
import frc.robot.commands.AutoFeeder;
import frc.robot.commands.FollowWaypoints;
import frc.robot.commands.SetAcquirerState;
import frc.robot.commands.SetStartPosition;
import frc.robot.commands.ToggleAcquirerPiston;
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
 * Autonomous command sequence moving the robot from the initiation line to the
 * right trench, and then into shooting position.
 */
public class InitiationLineToRightTrenchAuto extends SequentialCommandGroup {
  /**
   * Creates a new InitiationLineToRightTrenchAuto.
   */
  public InitiationLineToRightTrenchAuto(Drive drive, Acquirer acquirer, Feeder feeder, BallCounter ballCounter,
      ShooterRPM shooterRPM, Turret turret, LimelightVision limelightVision, AcquirerPiston acquirerPiston) {
    super(new SetStartPosition(drive, new Pose2d(3.473, -7.2, new Rotation2d(0))), 
          new SetAcquirerState(acquirerPiston, State.EXTEND), 
          new FollowWaypoints(drive,
                              // Starting pose
                              new Pose2d(3.43, -7.2, new Rotation2d(0)),
                              // Waypoints
                              List.of(new Translation2d(5.068, -6.809)),
                              // Ending pose
                              new Pose2d(6.4, -7.3, new Rotation2d(Math.toRadians(-50))),
                              // Drive forward
                              false)
            .alongWith(new AcquireNumberOfBalls(acquirer, ballCounter).withRelativeCount(2).withTimeout(5))
            .alongWith(new AutoFeeder(ballCounter, feeder))
            .andThen(new SetAcquirerState(acquirerPiston, State.RETRACT)),
          new FollowWaypoints(drive,
                              // Starting pose
                              new Pose2d(6.4, -7.3, new Rotation2d(Math.toRadians(-50))),
                              // Waypoints
                              List.of(new Translation2d(4.648, -4.236)),
                              // Ending pose
                              new Pose2d(4.549, -2.567, new Rotation2d(Math.toRadians(-45))),
                              // Drive backward
                              true)
            .alongWith(new TurnTurretToAngle(turret, 100)),
          new AutoShootSequence(4000, shooterRPM, turret, feeder, acquirer, ballCounter, limelightVision));
  }
}
