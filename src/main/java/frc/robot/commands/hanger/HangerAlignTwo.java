package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hardware.MuxedColorSensor;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.subsystems.SwerveDrive;

import java.util.logging.Level;
import java.util.logging.Logger;

import static frc.robot.Constants.HangerConstants.HANGER_ALIGN_ROTATION_VOLTAGE;

public class HangerAlignTwo extends CommandBase {
    private static final RobotLogger logger = new RobotLogger(HangerAlignTwo.class.getCanonicalName());
    private final SwerveDrive swerve;
    private final MuxedColorSensor colorSensor = MuxedColorSensor.getInstance();

    public HangerAlignTwo(SwerveDrive swerveDrive) {
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
        //rotate swerve around fixed sensor
        if (colorSensor.leftAlignSensorDetectsTape()){
            swerve.fixedLeftRotate(HANGER_ALIGN_ROTATION_VOLTAGE);
        }
        else if (colorSensor.rightAlignSensorDetectsTape()){
            swerve.fixedRightRotate(HANGER_ALIGN_ROTATION_VOLTAGE);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //stop
        swerve.stop();
        logger.info("Hanger Align Part Two Complete.");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return colorSensor.leftAlignSensorDetectsTape()&&
                colorSensor.rightAlignSensorDetectsTape();
    }
}
