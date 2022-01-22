package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.Limelight;

public class TestLimelight extends CommandBase {

    Limelight limelight;

    public TestLimelight(Limelight limelight) {
        this.limelight = limelight;
        System.out.println("TestInit");
    }

    @Override
    public void execute() {
        System.out.println("Running");

        SmartDashboard.putNumber("distance to target:", limelight.getDistanceToTarget());
        SmartDashboard.putNumber("(tuned) distance to target:", limelight.getTuned(limelight.getDistanceToTarget()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
