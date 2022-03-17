package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class ZeroHoodMotorCommand extends CommandBase {
    private RobotLogger logger = new RobotLogger(ZeroHoodMotorCommand.class.getCanonicalName());

    private final HoodSubsystem hood;

    /**
     * @param hood
     * zeros the hood motor and it's sensor
     */
    public ZeroHoodMotorCommand(HoodSubsystem hood) {
        this.hood=hood;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(hood);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logger.info("Zeroing Hood Motor");
        hood.hoodSlowReverse();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hood.stopHood();
        hood.zeroHoodMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("Limit", hood.isHoodLimitSwitchPressed());
        return hood.isHoodLimitSwitchPressed();
    }
}
