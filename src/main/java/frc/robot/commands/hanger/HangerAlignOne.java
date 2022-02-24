package frc.robot.commands.hanger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helper.Limelight;
import frc.robot.helper.MuxedColorSensor;
import frc.robot.subsystems.SwerveDrive;

import java.util.logging.Level;
import java.util.logging.Logger;

import static frc.robot.Constants.HangerConstants.HANGER_ALIGN_FEET_PER_SECOND;

public class HangerAlignOne extends CommandBase {
    private final SwerveDrive swerve;
    private final MuxedColorSensor colorSensor = MuxedColorSensor.getInstance();
    private static final Logger logger = Logger.getLogger(Limelight.class.getCanonicalName());

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
        //move swerve backwards
        swerve.drive(new ChassisSpeeds(0, HANGER_ALIGN_FEET_PER_SECOND, 0));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //stop
        swerve.drive(new ChassisSpeeds(0,0,0));
        logger.log(Level.INFO, "Hanger Align Part One Complete.");
        CommandScheduler.getInstance().schedule(new HangerAlignTwo(swerve));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return colorSensor.leftAlignSensorDetectsTape()||
                colorSensor.rightAlignSensorDetectsTape();
    }
}
