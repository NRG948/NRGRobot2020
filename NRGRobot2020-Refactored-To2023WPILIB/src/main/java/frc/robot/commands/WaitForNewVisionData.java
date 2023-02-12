// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.RaspberryPiVision;
// import frc.robot.utilities.Logger;

// public class WaitForNewVisionData extends CommandBase {
//   private RaspberryPiVision raspberryPiVision;
//   private int genCount;
//   /**
//    * Creates a new WaitForNewVisionData.
//    */
//   public WaitForNewVisionData(RaspberryPiVision raspberryPiVision) {
//     this.raspberryPiVision = raspberryPiVision;
//     addRequirements(this.raspberryPiVision);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     genCount = raspberryPiVision.getGenCount();
//     Logger.commandInit(this);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     Logger.commandEnd(this, interrupted);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return (raspberryPiVision.getGenCount() != this.genCount);
//   }
// }
