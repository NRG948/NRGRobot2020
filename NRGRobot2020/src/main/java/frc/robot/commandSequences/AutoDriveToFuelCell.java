/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandSequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDriveOnHeading;
import frc.robot.commands.AutoTurnToHeading;
import frc.robot.commands.SetRaspberryPiPipeline;
import frc.robot.commands.WaitForNewVisionData;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RaspberryPiVision;
import frc.robot.subsystems.RaspberryPiVision.PipelineRunner;
import frc.robot.vision.FuelCellTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoDriveToFuelCell extends SequentialCommandGroup {
  /**
   * Creates a new AutoDriveToFuelCell.
   */
  public AutoDriveToFuelCell(RaspberryPiVision raspberryPiVision, Drive drive) {

    super(new SetRaspberryPiPipeline(raspberryPiVision, PipelineRunner.FUEL_CELL),
        new WaitForNewVisionData(raspberryPiVision), 
        new InstantCommand(() -> {
          FuelCellTarget ballTarget = raspberryPiVision.getFuelCellTarget();
          if (ballTarget != null) {
            double distanceToTarget = ballTarget.getDistanceToTarget();
            double angleToTarget = ballTarget.getAngleToTarget();

            new AutoTurnToHeading(drive).withMaxPower(0.2).toHeading(drive.getHeading() + angleToTarget)
                .andThen(new AutoDriveOnHeading(drive).forMeters(distanceToTarget)).schedule();
          }
        }));
  }
}
