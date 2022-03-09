package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangerSubsystem;

public class HangerZeroRetract extends CommandBase {
    private HangerSubsystem hanger;

    private boolean leftMotorRunning = true;
    private boolean rightMotorRunning = true;

    public HangerZeroRetract(HangerSubsystem hanger) {
        this.hanger = hanger;
        addRequirements(hanger);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        hanger.retractLeftContinuouslyToZero();
        hanger.retractRightContinuouslyToZero();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (leftMotorRunning && hanger.isLeftHangerCurrentSpiking()){
            hanger.stopLeftMotor();
            leftMotorRunning = false;
        }

        if (rightMotorRunning && hanger.isRightHangerCurrentSpiking()){
            hanger.stopRightMotor();
            rightMotorRunning = false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Follows the Motors, so they don't get out of sync
        hanger.stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !rightMotorRunning && !leftMotorRunning;
    }

}