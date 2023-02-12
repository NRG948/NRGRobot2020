// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.RaspberryPiVision;
// import frc.robot.subsystems.RaspberryPiVision.PipelineRunner;
// import frc.robot.utilities.Logger;

// /**
//  * Add your docs here.
//  */
// public class SetRaspberryPiPipeline extends CommandBase {
//     private final RaspberryPiVision raspPi;
//     private final PipelineRunner runner;
    
//     public SetRaspberryPiPipeline(RaspberryPiVision raspPi, PipelineRunner runner) {
//         this.raspPi = raspPi;
//         this.runner = runner;
//         addRequirements(raspPi);
//     }

//     @Override
//     public void initialize() {
//         Logger.commandInit(this, runner.getName());
//         raspPi.setPipelineRunner(runner);
//     }

//     @Override
//     public void execute() {
//     }

//     @Override
//     public void end(boolean interrupted) {
//         Logger.commandEnd(this, interrupted);
//     }

//     @Override
//     public boolean isFinished() {
//         return true;
//     }
// }
