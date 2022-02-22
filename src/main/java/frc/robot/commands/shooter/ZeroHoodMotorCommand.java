package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

public class ZeroHoodMotorCommand extends CommandBase {

    private final FlywheelSubsystem flywheel;

    /**
     * @param subsystem
     * zeros the hood motor and it's sensor
     */
    public ZeroHoodMotorCommand(FlywheelSubsystem subsystem) {
        flywheel = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        flywheel.hoodSlowReverse();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        flywheel.stopHood();
        flywheel.zeroHoodMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return flywheel.isHoodLimitSwitchPressed();
    }
}
