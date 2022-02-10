package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.Limelight;

public class TestLimelight extends CommandBase {

    @Override
    public void execute() {
        SmartDashboard.putNumber("distance to target:", Limelight.getTunedDistanceToTarget());
        SmartDashboard.putNumber("(tuned) distance to target:", Limelight.getDistanceToTarget());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
