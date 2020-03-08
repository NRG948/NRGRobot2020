/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.AcquireNumberOfBalls;
import frc.robot.commands.AutoFeeder;
import frc.robot.commands.DriveToFuelCell;
import frc.robot.commands.SetAcquirerState;
import frc.robot.commands.SetRaspberryPiPipeline;
import frc.robot.commands.WaitForNewVisionData;
import frc.robot.subsystems.AcquirerPistons.State;
import frc.robot.subsystems.RaspberryPiVision.PipelineRunner;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoDriveToFuelCell extends SequentialCommandGroup {
  /**
   * Creates a new AutoDriveToFuelCell.
   */
  public AutoDriveToFuelCell(RobotSubsystems subsystems, int ballCount) {

    super(new SetRaspberryPiPipeline(subsystems.raspPi, PipelineRunner.FUEL_CELL)
          .alongWith(new SetAcquirerState(subsystems.acquirerPiston, State.EXTEND)),
        new WaitForNewVisionData(subsystems.raspPi), 
        new DriveToFuelCell(subsystems.drive, subsystems.raspPi)
          .raceWith(new AutoFeeder(subsystems.ballCounter, subsystems.feeder),
                    new AcquireNumberOfBalls(subsystems.acquirer, subsystems.ballCounter).withRelativeCount(ballCount)),
        new SetAcquirerState(subsystems.acquirerPiston, State.RETRACT)
    );
  }
}
