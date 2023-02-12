// package frc.robot.commandSequences;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.RobotSubsystems;
// import frc.robot.commands.AcquireNumberOfBalls;
// import frc.robot.commands.AutoFeeder;
// import frc.robot.commands.DriveToFuelCell;
// import frc.robot.commands.SetAcquirerState;
// import frc.robot.commands.SetRaspberryPiPipeline;
// import frc.robot.commands.WaitForNewVisionData;
// import frc.robot.subsystems.AcquirerPistons.State;
// import frc.robot.subsystems.RaspberryPiVision.PipelineRunner;
// import frc.robot.utilities.Logger;

// /**
//  * Autonomous command sequence that drives to and acquries the nearest fuel
//  * cell using vision processing input from the Raspberry Pi.
//  */
// public class AutoDriveToFuelCell extends SequentialCommandGroup {
//   /**
//    * Creates a new AutoDriveToFuelCell.
//    */
//   public AutoDriveToFuelCell(RobotSubsystems subsystems, int ballCount) {

//     super(new SetRaspberryPiPipeline(subsystems.raspPi, PipelineRunner.FUEL_CELL)
//           .alongWith(new SetAcquirerState(subsystems.acquirerPiston, State.EXTEND)),
//         new WaitForNewVisionData(subsystems.raspPi), 
//         new DriveToFuelCell(subsystems.drive, subsystems.raspPi)
//           .raceWith(new AutoFeeder(subsystems.ballCounter, subsystems.feeder),
//                     new AcquireNumberOfBalls(subsystems.acquirer, subsystems.ballCounter).withRelativeCount(ballCount)),
//         new SetAcquirerState(subsystems.acquirerPiston, State.RETRACT)
//     );
//   }

//   @Override
//   public void initialize(){
//     Logger.commandInit(this);
//     super.initialize();
//   }
  
//   @Override
//   public void end(boolean interrupted){
//     super.end(interrupted);
//     Logger.commandEnd(this, interrupted);
//   }
  
// }
