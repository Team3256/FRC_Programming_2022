package frc.robot.commands.transfer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class TransferShootForward extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final TransferSubsystem transferSubsystem;

    /**
     * If manually controlled, set as not interruptable, so that auto index commands don't take control.
     * @param transferSubsystem Transfer Subsystem
     */
    public TransferShootForward(TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem) {
        this.transferSubsystem = transferSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(transferSubsystem);
    }

    @Override
    public void execute() {
        double velocity = shooterSubsystem.getTargetVelocity();
        SmartDashboard.putBoolean("Ready to Shoot", shooterSubsystem.isAtSetPoint(velocity));
        if (shooterSubsystem.isAtSetPoint(velocity)) {
            transferSubsystem.forward();
        } else {
            transferSubsystem.off();
        }
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
