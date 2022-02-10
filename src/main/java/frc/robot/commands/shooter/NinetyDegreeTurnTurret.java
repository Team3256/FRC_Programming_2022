package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class NinetyDegreeTurnTurret extends CommandBase {
    private final TurretSubsystem turretSubsystem;

    public NinetyDegreeTurnTurret (TurretSubsystem turretSubsystem){
        this.turretSubsystem = turretSubsystem;
    }
    @Override
    public void initialize() {
        turretSubsystem.ninetyDegreeTurn();
        System.out.println("Beginning turret turning program.");
    }

    @Override
    public void end(boolean interrupted){
        turretSubsystem.stop();
        System.out.println("Ending turret turning program.");
    }
}
