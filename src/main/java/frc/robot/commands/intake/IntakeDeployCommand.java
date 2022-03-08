package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDeployCommand extends CommandBase {
    IntakeSubsystem intake;
    public IntakeDeployCommand(IntakeSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.solenoidExtend();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
