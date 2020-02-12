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
import frc.robot.commands.MaintainShooterRPM;
import frc.robot.commands.SetApproximateShooterRPM;
import frc.robot.commands.WaitForMinRPM;
import frc.robot.subsystems.ShooterRPM;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShootSequence extends SequentialCommandGroup {
  /**
   * Creates a new ShootAtMinRPM.
   */
  public AutoShootSequence(double rpm, ShooterRPM shooterRPM) {

    super( 
      // Parallel, Raise a close to target
      new ParallelCommandGroup(new SetApproximateShooterRPM(rpm * .9, shooterRPM)),
      // Parallel, Is turret aimed?
      // Wait for ball
      // Is shoot button pressed?
      // Start ramping up rpm
      new InstantCommand(() -> { shooterRPM.setFlyWheel(1); }),
      // Are you at target rpm?
      new WaitForMinRPM(rpm, shooterRPM)
      // Release Ball
      // Feed balls through indexer
    );
  }
}
