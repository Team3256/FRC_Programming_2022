package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDShooterSubsystem;

public class PIDShooterCommand extends CommandBase {
    private PIDShooterSubsystem m_subsystem;

    public PIDShooterCommand(PIDShooterSubsystem subsystem) {
        m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.setSpeed(0);
    }

    public void setVelocity(double velocity) {
        m_subsystem.setSpeed(velocity);
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
