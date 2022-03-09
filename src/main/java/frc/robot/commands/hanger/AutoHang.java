package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.HangerSubsystem;
import static frc.robot.Constants.HangerConstants.*;

public class AutoHang extends SequentialCommandGroup {
    HangerSubsystem hanger;
    public AutoHang(HangerSubsystem hanger) {
        this.hanger = hanger;
        addCommands(
                new HangerZeroRetract(hanger),
                new WaitCommand(RETRACT_WAIT),
                new HangerPartial(hanger),
                new WaitCommand(PARTIAL_EXTEND_WAIT),
                new HangerPneumaticSlant(hanger),
                new HangerExtend(hanger),
                new WaitCommand(EXTEND_WAIT),
                new HangerPneumaticUpright(hanger),
                new HangerZeroRetract(hanger),
                new WaitCommand(RETRACT_WAIT),
                new HangerPartial(hanger),
                new WaitCommand(PARTIAL_EXTEND_WAIT),
                new HangerPneumaticSlant(hanger),
                new HangerExtend(hanger),
                new WaitCommand(EXTEND_WAIT),
                new HangerPneumaticUpright(hanger),
                new HangerZeroRetract(hanger),
                new WaitCommand(RETRACT_WAIT)
        );
    }

}
