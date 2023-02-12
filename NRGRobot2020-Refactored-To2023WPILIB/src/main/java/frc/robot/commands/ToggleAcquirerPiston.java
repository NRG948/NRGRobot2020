// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.AcquirerPistons;
// import frc.robot.utilities.Logger;

// /**
//  * A command which toggles (inverts) the state of the acquirer pistons.
//  * 
//  * That is, if the acquirer is currently up/stowed, this command will extend it.
//  * If the acquirer is currently extended, this command will stow it.
//  */
// public class ToggleAcquirerPiston extends CommandBase {

//   private AcquirerPistons acquirerPiston;

//   /**
//    * Creates a new ToggleAcquirerPiston command.
//    */
//   public ToggleAcquirerPiston(AcquirerPistons acquirerPiston) {
//     this.acquirerPiston = acquirerPiston;
//     addRequirements(acquirerPiston);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() { 
//     Logger.commandInit(this);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     acquirerPiston.toggleState();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     Logger.commandEnd(this, interrupted, "state: " + acquirerPiston.getState().toString());
//   }

//   // Returns true when the command should end, which is immediately.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }
