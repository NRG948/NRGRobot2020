// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.AcquirerPistons;
// import frc.robot.subsystems.AcquirerPistons.State;
// import frc.robot.utilities.Logger;

// public class SetAcquirerState extends CommandBase {
//   private AcquirerPistons acquirerPiston;
//   private State state;
//   /**
//    * Creates a new SetAcquirerState.
//    */
//   public SetAcquirerState(AcquirerPistons acquirerPiston, State state) {
//     this.acquirerPiston = acquirerPiston;
//     this.state = state;
//     addRequirements(acquirerPiston);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     Logger.commandInit(this, state.toString());
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     acquirerPiston.setState(state);
//   }
  
//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
  
//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//       Logger.commandEnd(this, interrupted);
//   }
// }