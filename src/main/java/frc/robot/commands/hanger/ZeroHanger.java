package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.HangerSubsystem;

public class ZeroHanger extends CommandBase {
    HangerSubsystem hanger;

    public ZeroHanger(HangerSubsystem hanger){
        addRequirements(hanger);
        this.hanger=hanger;
    }

    @Override
    public void initialize() {
        hanger.zero();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
