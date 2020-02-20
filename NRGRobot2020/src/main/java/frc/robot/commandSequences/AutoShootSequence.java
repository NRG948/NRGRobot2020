/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandSequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoFeedToShooter;
import frc.robot.commands.AutoTurret;
import frc.robot.commands.MaintainShooterRPM;
import frc.robot.commands.SetApproximateShooterRPM;
import frc.robot.commands.WaitForBallReady;
import frc.robot.commands.WaitForMinRPM;
import frc.robot.subsystems.Acquirer;
import frc.robot.subsystems.BallCounter;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShootSequence extends SequentialCommandGroup {
  /**
   * Creates a new ShootAtMinRPM.
   */
  public AutoShootSequence(double rpm, ShooterRPM shooterRPM, Turret turret, Feeder feeder, Acquirer acquirer, BallCounter ballCounter) {

    super( 
      // Parallel, Raise a close to target and start Turret PID.
      new SetApproximateShooterRPM(rpm * .9, shooterRPM)
        // .alongWith(new AutoTurret(turret))
        .andThen(new WaitForBallReady(ballCounter))
      // Start ramping up rpm
        .andThen(new InstantCommand(() -> { shooterRPM.setFlyWheel(1); }))
      // Are you at target rpm?
        .andThen(new WaitForMinRPM(rpm, shooterRPM))
      // Release Ball
        .andThen(new AutoFeedToShooter(acquirer, feeder, ballCounter))
      // Feed balls through indexer
    );
  }
}
