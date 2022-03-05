package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.Limelight;
import frc.robot.subsystems.TurretSubsystem ;

public class EnableDisableLimelight extends CommandBase {
    TurretSubsystem turretSubsystem;

    private EnableDisableLimelight() {
        turretSubsystem = new TurretSubsystem();
    }

    Limelight.enable();

    Limelight.disable();
}
