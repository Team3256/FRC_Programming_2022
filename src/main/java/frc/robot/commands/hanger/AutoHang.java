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
                new HangerRetract(hanger),
                new WaitCommand(WAIT_TIME),
                new HangerPartial(hanger),
                new WaitCommand(WAIT_TIME),
                new HangerPneumaticSlant(hanger),
                new WaitCommand(WAIT_TIME),
                new HangerExtend(hanger),
                new WaitCommand(WAIT_TIME),
                new HangerPneumaticUpright(hanger),
                new WaitCommand(WAIT_TIME),
                new HangerRetract(hanger),
                new WaitCommand(WAIT_TIME),
                new HangerPartial(hanger),
                new WaitCommand(WAIT_TIME),
                new HangerPneumaticSlant(hanger),
                new WaitCommand(WAIT_TIME),
                new HangerExtend(hanger),
                new WaitCommand(WAIT_TIME),
                new HangerPneumaticUpright(hanger),
                new WaitCommand(WAIT_TIME),
                new HangerRetract(hanger),
                new WaitCommand(WAIT_TIME)
                //TODO: add friction break
        );
    }

}
