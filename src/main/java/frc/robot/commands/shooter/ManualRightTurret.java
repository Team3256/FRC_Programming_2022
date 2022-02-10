package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class ManualRightTurret extends CommandBase {
    private final TurretSubsystem turretSubsystem;

    public ManualRightTurret(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
    }

    @Override
    public void initialize(){
        turretSubsystem.manualRight();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stop();
    }
}
