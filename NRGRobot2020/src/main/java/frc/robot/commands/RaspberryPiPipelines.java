package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RaspberryPiVision;
import frc.robot.subsystems.RaspberryPiVision.PipelineRunner;

/**
 * Add your docs here.
 */
public class RaspberryPiPipelines extends CommandBase {
    private final RaspberryPiVision raspPi;
    private final PipelineRunner runner;
    public RaspberryPiPipelines(RaspberryPiVision raspPi, PipelineRunner runner) {
        this.raspPi = raspPi;
        this.runner = runner;
        addRequirements(raspPi);
    }

    @Override
    public void initialize() {
        raspPi.setPipelineRunner(runner);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
