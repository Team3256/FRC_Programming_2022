package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.LimelightAutocorrectStepCommand;
import frc.robot.helper.MuxedColorSensor;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;

import static frc.robot.Constants.HangerConstants.TAPE_COLOR;

public class HangerAlignOne extends CommandBase {
    private final SwerveDrive swerve;
    private final MuxedColorSensor colorSensor = MuxedColorSensor.getInstance();
    public HangerAlignOne(SwerveDrive swerveDrive) {
        swerve = swerveDrive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(new HangerAlignTwo(swerve));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return colorSensor.colorsMatch(colorSensor.getLeftAlignSensorColor(), TAPE_COLOR)||
                colorSensor.colorsMatch(colorSensor.getRightAlignSensorColor(), TAPE_COLOR);
    }
}
