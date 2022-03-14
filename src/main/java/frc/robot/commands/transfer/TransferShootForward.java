package frc.robot.commands.transfer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferSubsystem;

public class TransferShootForward extends CommandBase {

    private final TransferSubsystem transferSubsystem;

    /**
     * If manually controlled, set as not interruptable, so that auto index commands don't take control.
     * @param transferSubsystem Transfer Subsystem
     */
    public TransferShootForward(TransferSubsystem transferSubsystem) {
        this.transferSubsystem = transferSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(transferSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        transferSubsystem.forwardShoot();
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.off();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
