package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOn extends CommandBase {
    private final IntakeSubsystem intake;
    public IntakeOn(IntakeSubsystem subsystem) {
        intake = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        intake.on(1);
    }
    @Override
    public void execute() {}
    @Override
    public void end(boolean interrupted) {
        intake.off();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
