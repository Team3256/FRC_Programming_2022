package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangerSubsystem;

public class HangerRetract extends CommandBase {
    private HangerSubsystem hanger;
    public HangerRetract(HangerSubsystem hanger) {
        this.hanger = hanger;
        addRequirements(hanger);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {hanger.retractContinuously();}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hanger.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return hanger.isCurrentLimitReached();
    }

}